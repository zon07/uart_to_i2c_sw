/*
 * i2c_master.c
 *
 *  Created on: 23 июн. 2025 г.
 *      Author: user
 */

#include "i2c_driver_master.h"

#include "mik32_memory_map.h"
#include "power_manager.h"
#include "pad_config.h"
#include "i2c.h"
#include "epic.h"

/* Выбор канала 0 или 1*/
#define BSP_I2C_CHANNEL		(0U)


// Настройки I2C (аппаратные)
#if (BSP_I2C_CHANNEL == 0)
    #define DRV_I2C_MASTER_EPIC_LINE    		(EPIC_LINE_I2C_0_S)
	#define DRV_I2C_MASTER_PM_CLOCK_APB_P 		(PM_CLOCK_APB_P_I2C_0_M)

	#define DRV_I2C_MASTER_PORT_CFG				(PAD_CONFIG->PORT_0_CFG)
	#define DRV_I2C_MASTER_PORT_DS				(PAD_CONFIG->PORT_0_DS)
	#define DRV_I2C_MASTER_PORT_PUPD			(PAD_CONFIG->PORT_0_PUPD)

	#define DRV_I2C_MASTER_SDA_PIN				(9)
	#define DRV_I2C_MASTER_SCL_PIN				(10)

    static I2C_TypeDef *hi2c = I2C_0;
#else
    #define DRV_I2C_MASTER_EPIC_LINE    		(EPIC_LINE_I2C_1_S)
	#define DRV_I2C_MASTER_PM_CLOCK_APB_P		(PM_CLOCK_APB_P_I2C_1_M)

	#define DRV_I2C_MASTER_PORT_CFG				(PAD_CONFIG->PORT_1_CFG)
	#define DRV_I2C_MASTER_PORT_DS				(PAD_CONFIG->PORT_1_DS)
	#define DRV_I2C_MASTER_PORT_PUPD			(PAD_CONFIG->PORT_1_PUPD)

	#define DRV_I2C_MASTER_SDA_PIN				(12)
	#define DRV_I2C_MASTER_SCL_PIN				(13)

    static I2C_TypeDef *hi2c = I2C_1;

#endif

#define I2C_TRANSACTION_TIMEOUT_MS    (100) // Таймаут транзакции в мс

typedef struct {
	volatile uint16_t cmdPos;
	volatile uint16_t dataPos;
	volatile bool    isReading;
	volatile uint32_t startTime;
	volatile bool    timeout;
} I2C_TransactionState_t;

// Очередь и текущая транзакция
static QueueHandle_t xI2C_Queue = NULL;
static I2C_Master_Transaction_t *currentTrans;
static I2C_TransactionState_t i2cState = {0};

// Приватные функции
static void I2C_Task(void *pvParameters);
static void I2C_StartTransaction(void);
static void I2C_RecoverBus(void);

// Инициализация драйвера
void Drv_I2C_Master_Init(void)
{
    xI2C_Queue = xQueueCreate(1, sizeof(I2C_Master_Transaction_t*)); // Очередь на 1 транзакцию

    xTaskCreate(I2C_Task, "I2C", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 2, NULL); // Создаем задачу

    PM->CLK_APB_P_SET = DRV_I2C_MASTER_PM_CLOCK_APB_P;

    DRV_I2C_MASTER_PORT_DS 	&= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN);
    DRV_I2C_MASTER_PORT_DS   |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b11); 	// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
    DRV_I2C_MASTER_PORT_PUPD &= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN);
    DRV_I2C_MASTER_PORT_PUPD |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b00);	// Подтяжка Pull-Up

	DRV_I2C_MASTER_PORT_DS 	&= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN);
	DRV_I2C_MASTER_PORT_DS   |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b11); 	// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
	DRV_I2C_MASTER_PORT_PUPD &= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN);
	DRV_I2C_MASTER_PORT_PUPD |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b00); 	// Подтяжка Pull-Up

	DRV_I2C_MASTER_PORT_CFG &= ~(PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN) | PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN));
	DRV_I2C_MASTER_PORT_CFG |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b01);	// SDA
	DRV_I2C_MASTER_PORT_CFG |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b01); 	// SCL

	/* Выключаем модуль I2C*/
	hi2c->CR1 &= ~I2C_CR1_PE_M;

    /* Аналоговый фильтр
     * 0 - выключен,
     * 1 - включить;
     * */
    hi2c->CR1 &= ~I2C_CR1_ANFOFF_M;
    hi2c->CR1 |= (1 << I2C_CR1_ANFOFF_S);

    /* Цифровой фильтр
     * 0b0000 - выключен,
     * 0b0001 - 1 такт;
     * -----
     * 0b1111 - 15 тактов
     * */
    hi2c->CR1 &= ~I2C_CR1_DNF_M;
    hi2c->CR1 |= I2C_CR1_DNF(0);

    /* Настройка частоты и временных параметров */
    hi2c->TIMINGR = 0;

    hi2c->TIMINGR |= I2C_TIMINGR_PRESC(0); // Делитель 2

    hi2c->TIMINGR |= I2C_TIMINGR_SCLDEL(12);
    hi2c->TIMINGR |= I2C_TIMINGR_SDADEL(10);
    hi2c->TIMINGR |= I2C_TIMINGR_SCLH(27);
    hi2c->TIMINGR |= I2C_TIMINGR_SCLL(27);

    /* Растягивание. В режиме Master должно быть 0 */
    hi2c->CR1 &= ~I2C_CR1_NOSTRETCH_M;

    /* Включение прерываний*/
    hi2c->CR1 = I2C_CR1_ERRIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TCIE_M | I2C_CR1_TXIE_M | I2C_CR1_RXIE_M;

    /* Сбросить флаги прерываний*/
    hi2c->ICR = I2C_ICR_OVRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_BERRCF_M | I2C_ICR_STOPCF_M | I2C_ICR_NACKCF_M | I2C_ICR_ADDRCF_M;

    /* Включение прерываний в EPIC*/
    EPIC->MASK_EDGE_SET = EPIC_LINE_M(DRV_I2C_MASTER_EPIC_LINE);

    /* Включаем модуль I2C*/
    hi2c->CR1 |= I2C_CR1_PE_M;
}


// Отправка транзакции в очередь
bool Drv_I2C_Master_SendTransaction(I2C_Master_Transaction_t *trans, TickType_t timeout)
{
    if (trans == NULL || trans->completionSem == NULL || trans->devAddr > 0x7F)
    {
        return false;
    }

    switch (trans->opMode)
    {
        case I2C_OP_WRITE_THEN_READ:
            if (trans->writeData == NULL || trans->readData == NULL ||
                trans->writeDataLen == 0 || trans->writeDataLen > I2C_WRITE_DATA_MAX_LEN ||
                trans->readDataLen == 0 || trans->readDataLen > I2C_READ_DATA_MAX_LEN)
            {
                return false;
            }
            break;

        case I2C_OP_WRITE_ONLY:
            if (trans->writeData == NULL || trans->writeDataLen == 0 ||
                trans->writeDataLen > I2C_WRITE_DATA_MAX_LEN)
            {
                return false;
            }
            break;

        case I2C_OP_READ_ONLY:
            if (trans->readData == NULL || trans->readDataLen == 0 ||
                trans->readDataLen > I2C_READ_DATA_MAX_LEN)
            {
                return false;
            }
            if (trans->writeData != NULL)
            {
                return false;
            }
            break;

        default:
            return false;
    }

    if (xQueueSend(xI2C_Queue, &trans, timeout) != pdPASS)
    {
        return false; // Очередь занята
    }

    // Ожидаем завершения (именно из транзакции currentTrans)
    if (xSemaphoreTake(trans->completionSem, timeout) == pdPASS)
    {
        return trans->result;
    }

    return false; // Таймаут
}


// Задача FreeRTOS для обработки транзакций
static void I2C_Task(void *pvParameters)
{
    while (1)
    {
        if (xQueueReceive(xI2C_Queue, &currentTrans, portMAX_DELAY) == pdTRUE)
        {
            // Проверка флага BUSY перед началом транзакции
            if (hi2c->ISR & I2C_ISR_BUSY_M)
            {
                currentTrans->result = false;
                I2C_RecoverBus();
                xSemaphoreGive(currentTrans->completionSem);
                continue;
            }

            I2C_StartTransaction();
        }
    }
}

// Запуск транзакции (аппаратно-зависимый код)
static void I2C_StartTransaction(void)
{
    // Проверка флага BUSY перед началом транзакции
    if (hi2c->ISR & I2C_ISR_BUSY_M)
    {
        currentTrans->result = false;
        xSemaphoreGive(currentTrans->completionSem);
        return;
    }

    i2cState.cmdPos = 0;          // Сброс позиции команды
    i2cState.dataPos = 0;         // Сброс позиции данных
    i2cState.isReading = false;   // Сброс фазы чтения
    i2cState.startTime = xTaskGetTickCount();


    hi2c->CR2 &= ~(I2C_CR2_START_M | I2C_CR2_STOP_M | I2C_CR2_RD_WRN_M |
                   I2C_CR2_NBYTES_M | I2C_CR2_AUTOEND_M | I2C_CR2_RELOAD_M |
                   I2C_CR2_SADD_M);

    switch (currentTrans->opMode)
    {
        case I2C_OP_WRITE_THEN_READ:
            hi2c->CR2 = (currentTrans->devAddr << 1) |
                        I2C_CR2_NBYTES(currentTrans->writeDataLen) |
                        I2C_CR2_START_M;
            break;

        case I2C_OP_WRITE_ONLY:
            hi2c->CR2 = (currentTrans->devAddr << 1) |
                        I2C_CR2_NBYTES(currentTrans->writeDataLen) |
                        I2C_CR2_START_M;
            			// |I2C_CR2_AUTOEND_M;
            break;

        case I2C_OP_READ_ONLY:
            hi2c->CR2 = (currentTrans->devAddr << 1) |
                        I2C_CR2_NBYTES(currentTrans->readDataLen) |
                        I2C_CR2_START_M |
                        I2C_CR2_RD_WRN_M |
                        I2C_CR2_AUTOEND_M;
            break;
    }
}


static void I2C_RecoverBus(void)
{
    // 1. Выключаем I2C периферию
    hi2c->CR1 &= ~I2C_CR1_PE_M;

    // 2. Ждем минимум 3 такта APB шины для гарантированного сброса
    // (вставляем небольшую задержку)
    volatile uint32_t delay = 10; // ~10 тактов при 0 wait states
    while(delay--);

    // 3. Включаем I2C обратно
    hi2c->CR1 |= I2C_CR1_PE_M;

    // 4. Дополнительно: сбрасываем все флаги ошибок
    hi2c->ICR = I2C_ICR_OVRCF_M | I2C_ICR_ARLOCF_M |
                I2C_ICR_BERRCF_M | I2C_ICR_STOPCF_M |
                I2C_ICR_NACKCF_M | I2C_ICR_ADDRCF_M;
}


void Drv_I2C_Master_IRQ_Handler(BaseType_t *pxHigherPriorityTaskWoken)
{
	uint32_t isr = hi2c->ISR;
	bool transactionComplete = false;

    if ((isr & (I2C_ISR_NACKF_M | I2C_ISR_BERR_M | I2C_ISR_ARLO_M | I2C_ISR_OVR_M)) ||
        (xTaskGetTickCountFromISR() - i2cState.startTime) > pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS))
    {
        hi2c->CR2 |= I2C_CR2_STOP_M;

        hi2c->ICR = I2C_ICR_NACKCF_M | I2C_ICR_BERRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_OVRCF_M;

        I2C_RecoverBus();

        currentTrans->result = false;
        transactionComplete = true;
    }


	if (!transactionComplete)
	{
		switch (currentTrans->opMode)
		{
			case I2C_OP_WRITE_THEN_READ:
				if ((isr & I2C_ISR_TXIS_M) && (i2cState.cmdPos < currentTrans->writeDataLen) && !i2cState.isReading)
				{
					hi2c->TXDR = currentTrans->writeData[i2cState.cmdPos++];
				}

				if ((isr & I2C_ISR_TC_M) && (i2cState.cmdPos == currentTrans->writeDataLen) && !i2cState.isReading)
				{
					hi2c->CR2 &= ~(I2C_CR2_NBYTES_M | I2C_CR2_RD_WRN_M | I2C_CR2_AUTOEND_M);  // Сброс управляющих битов

					i2cState.isReading = true;  // Переход в фазу чтения
					hi2c->CR2 = (currentTrans->devAddr << 1) |
								I2C_CR2_NBYTES(currentTrans->readDataLen) |
								I2C_CR2_START_M |
								I2C_CR2_RD_WRN_M;
				}

				if ((isr & I2C_ISR_TC_M) && (i2cState.dataPos == currentTrans->readDataLen) && i2cState.isReading )
				{
					hi2c->CR2 |= I2C_CR2_STOP_M; // Явный STOP
					currentTrans->result = true;
					transactionComplete = true;
				}
				break;

			case I2C_OP_WRITE_ONLY:
				if (isr & I2C_ISR_TXIS_M)
				{
					if (i2cState.cmdPos < currentTrans->writeDataLen)
					{
						hi2c->TXDR = currentTrans->writeData[i2cState.cmdPos++];
					}
				}

				if ((isr & I2C_ISR_TC_M) )
				{
					if(true && (i2cState.cmdPos == currentTrans->writeDataLen))
					{
						hi2c->CR2 |= I2C_CR2_STOP_M; // Явный STOP
						currentTrans->result = true;
						transactionComplete = true;
					}
				}
				break;

			case I2C_OP_READ_ONLY:
				if ((isr & I2C_ISR_TC_M) && (i2cState.dataPos == currentTrans->readDataLen))
				{
					//hi2c->CR2 |= I2C_CR2_STOP_M; // Явный STOP
					currentTrans->result = true;
					transactionComplete = true;
				}
				break;
		}
		// Общая обработка приема данных
		if (isr & I2C_ISR_RXNE_M)
		{
		    if (i2cState.dataPos < currentTrans->readDataLen)
		    {
		        currentTrans->readData[i2cState.dataPos++] = hi2c->RXDR;
		    }
		}
	}

	// Уведомление о завершении
	if (transactionComplete)
	{
		xSemaphoreGiveFromISR(currentTrans->completionSem, pxHigherPriorityTaskWoken);
	}
}

