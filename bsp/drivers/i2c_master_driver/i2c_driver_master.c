/* i2c_master.c */

#include "i2c_driver_master.h"
#include "mik32_memory_map.h"
#include "power_manager.h"
#include "pad_config.h"
#include "i2c.h"
#include "dma_config.h"
#include "epic.h"
#include <string.h>

/* Выбор канала 0 или 1*/
#define BSP_I2C_CHANNEL		(0U)

// Настройки I2C (аппаратные)
#if (BSP_I2C_CHANNEL == 0)
    #define DRV_I2C_MASTER_EPIC_LINE    		(EPIC_LINE_I2C_0_S)
	#define DRV_I2C_MASTER_PM_CLOCK_APB_P 		(PM_CLOCK_APB_P_I2C_0_M)

	#define DRV_I2C_MASTER_DMA_REQUEST			(DMA_I2C_0_INDEX)

	#define DRV_I2C_MASTER_PORT_CFG				(PAD_CONFIG->PORT_0_CFG)
	#define DRV_I2C_MASTER_PORT_DS				(PAD_CONFIG->PORT_0_DS)
	#define DRV_I2C_MASTER_PORT_PUPD			(PAD_CONFIG->PORT_0_PUPD)

	#define DRV_I2C_MASTER_SDA_PIN				(9)
	#define DRV_I2C_MASTER_SCL_PIN				(10)

    static I2C_TypeDef *hi2c = I2C_0;
#else
    #define DRV_I2C_MASTER_EPIC_LINE    		(EPIC_LINE_I2C_1_S)
	#define DRV_I2C_MASTER_PM_CLOCK_APB_P		(PM_CLOCK_APB_P_I2C_1_M)

	#define DRV_I2C_MASTER_DMA_REQUEST			(DMA_I2C_1_INDEX)

	#define DRV_I2C_MASTER_PORT_CFG				(PAD_CONFIG->PORT_1_CFG)
	#define DRV_I2C_MASTER_PORT_DS				(PAD_CONFIG->PORT_1_DS)
	#define DRV_I2C_MASTER_PORT_PUPD			(PAD_CONFIG->PORT_1_PUPD)

	#define DRV_I2C_MASTER_SDA_PIN				(12)
	#define DRV_I2C_MASTER_SCL_PIN				(13)

    static I2C_TypeDef *hi2c = I2C_1;
#endif


#define I2C_DMA_RX_CHANNEL         (1U)    // Канал DMA для приема
#define I2C_DMA_TX_CHANNEL         (2U)    // Канал DMA для передачи

#define I2C_TRANSACTION_TIMEOUT_MS    (100) // Таймаут транзакции в мс

typedef struct {
	volatile uint16_t cmdPos;
	volatile uint16_t dataPos;
	volatile bool    isReading;
	volatile uint32_t startTime;
	volatile bool    timeout;
} I2C_TransactionState_t;

// Статический указатель на текущую транзакцию
static I2C_Master_Transaction_t *currentTrans = NULL;
static I2C_TransactionState_t i2cState = {0};
static SemaphoreHandle_t i2cMutex = NULL;
static SemaphoreHandle_t completionSem = NULL;

// DMA буфер
static uint8_t dma_rx_buffer[I2C_READ_DATA_MAX_LEN];

// Приватные функции
static void I2C_StartTransaction(void);
static void I2C_RecoverBus(void);
static bool I2C_ValidateTransaction(I2C_Master_Transaction_t *trans);
static void I2C_DMA_Init(void);
static bool I2C_StartDmaReceive(uint8_t* buffer, uint16_t length);


// Инициализация драйвера
void Drv_I2C_Master_Init(void)
{
    // Создаем мьютекс для защиты доступа к I2C
    i2cMutex = xSemaphoreCreateMutex();
    completionSem = xSemaphoreCreateBinary();

    PM->CLK_APB_P_SET = DRV_I2C_MASTER_PM_CLOCK_APB_P;

    // Конфигурация пинов (остается без изменений)
    DRV_I2C_MASTER_PORT_DS 	&= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN);
    DRV_I2C_MASTER_PORT_DS   |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b11);
    DRV_I2C_MASTER_PORT_PUPD &= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN);
    DRV_I2C_MASTER_PORT_PUPD |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b00);

	DRV_I2C_MASTER_PORT_DS 	&= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN);
	DRV_I2C_MASTER_PORT_DS   |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b11);
	DRV_I2C_MASTER_PORT_PUPD &= ~PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN);
	DRV_I2C_MASTER_PORT_PUPD |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b00);

	DRV_I2C_MASTER_PORT_CFG &= ~(PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SDA_PIN) | PAD_CONFIG_PIN_M(DRV_I2C_MASTER_SCL_PIN));
	DRV_I2C_MASTER_PORT_CFG |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SDA_PIN, 0b01);
	DRV_I2C_MASTER_PORT_CFG |= PAD_CONFIG_PIN(DRV_I2C_MASTER_SCL_PIN, 0b01);

	/* Выключаем модуль I2C*/
	hi2c->CR1 &= ~I2C_CR1_PE_M;

    /* Аналоговый фильтр */
    hi2c->CR1 &= ~I2C_CR1_ANFOFF_M;
    hi2c->CR1 |= (1 << I2C_CR1_ANFOFF_S);

    /* Цифровой фильтр */
    hi2c->CR1 &= ~I2C_CR1_DNF_M;
    hi2c->CR1 |= I2C_CR1_DNF(0);

    /* Настройка частоты и временных параметров */
    hi2c->TIMINGR = 0;
    hi2c->TIMINGR |= I2C_TIMINGR_PRESC(0);
    hi2c->TIMINGR |= I2C_TIMINGR_SCLDEL(12);
    hi2c->TIMINGR |= I2C_TIMINGR_SDADEL(10);
    hi2c->TIMINGR |= I2C_TIMINGR_SCLH(27);
    hi2c->TIMINGR |= I2C_TIMINGR_SCLL(27);

    /* Растягивание */
    hi2c->CR1 &= ~I2C_CR1_NOSTRETCH_M;

    /* Включение прерываний*/
    hi2c->CR1 = I2C_CR1_ERRIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TCIE_M | I2C_CR1_TXIE_M | I2C_CR1_RXIE_M;

    /* Сбросить флаги прерываний*/
    hi2c->ICR = I2C_ICR_OVRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_BERRCF_M | I2C_ICR_STOPCF_M | I2C_ICR_NACKCF_M | I2C_ICR_ADDRCF_M;

    /* Включение прерываний в EPIC*/
    EPIC->MASK_EDGE_SET = EPIC_LINE_M(DRV_I2C_MASTER_EPIC_LINE);

    /* Включаем модуль I2C*/
    hi2c->CR1 |= I2C_CR1_PE_M;

    /* Инициализация DMA */
    I2C_DMA_Init();
}

// Инициализация DMA для I2C
static void I2C_DMA_Init(void)
{
    PM->CLK_AHB_SET |= PM_CLOCK_AHB_DMA_M;  // Включить тактирование DMA
    EPIC->MASK_LEVEL_SET |= EPIC_LINE_M(EPIC_DMA_INDEX);
}

// Запуск приема через DMA
static bool I2C_StartDmaReceive(uint8_t* buffer, uint16_t length)
{
    if ((DMA_CONFIG->CONFIG_STATUS & DMA_STATUS_READY(I2C_DMA_RX_CHANNEL)) == 0)
    {
        return false;  // Канал занят
    }

    DMA_CHANNEL_TypeDef *dmaChannelInst = &DMA_CONFIG->CHANNELS[I2C_DMA_RX_CHANNEL];

    dmaChannelInst->CFG &= ~DMA_CH_CFG_ENABLE_M;

    // Настройка DMA канала для приема
    dmaChannelInst->SRC = (uint32_t)&hi2c->RXDR;  // Источник - регистр данных I2C
    dmaChannelInst->DST = (uint32_t)buffer;       // Приемник - буфер в памяти
    dmaChannelInst->LEN = length - 1;             // Длина данных

    // Конфигурация канала для приема
    dmaChannelInst->CFG =
        DMA_CH_CFG_PRIOR_HIGH_M |                 // Высокий приоритет
        DMA_CH_CFG_READ_MODE_PERIPHERY_M |        // Чтение из периферии
        DMA_CH_CFG_WRITE_MODE_MEMORY_M |          // Запись в память
        DMA_CH_CFG_READ_NO_INCREMENT_M |          // Фиксированный адрес источника
        DMA_CH_CFG_WRITE_INCREMENT_M |            // Инкремент адреса приемника
        DMA_CH_CFG_READ_SIZE_BYTE_M |             // Размер чтения - байт
        DMA_CH_CFG_WRITE_SIZE_BYTE_M |            // Размер записи - байт
        DMA_CH_CFG_READ_REQUEST(DRV_I2C_MASTER_DMA_REQUEST) |  // Запрос DMA для I2C (чтение)
        DMA_CH_CFG_WRITE_REQUEST(DRV_I2C_MASTER_DMA_REQUEST) | // Запрос DMA для I2C (запись)
        DMA_CH_CFG_IRQ_EN_M;                      // Разрешить прерывание канала

    DMA_CONFIG->CONFIG_STATUS =
        DMA_CONFIG_GLOBAL_IRQ_ENA_M |             // Разрешить глобальные прерывания DMA
        DMA_CONFIG_CURRENT_VALUE_M;               // Разрешить чтение текущих значений

    // Включаем DMA прием в I2C
    hi2c->CR1 |= I2C_CR1_RXDMAEN_M;

    dmaChannelInst->CFG |= DMA_CH_CFG_ENABLE_M;
    return true;
}

// Валидация транзакции
static bool I2C_ValidateTransaction(I2C_Master_Transaction_t *trans)
{
    if (trans == NULL || trans->devAddr > 0x7F)
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

    return true;
}

// Отправка транзакции (блокирующая)
bool Drv_I2C_Master_SendTransaction(I2C_Master_Transaction_t *trans, TickType_t timeout)
{
    if (!I2C_ValidateTransaction(trans))
    {
        return false;
    }

    // Захватываем мьютекс I2C
    if (xSemaphoreTake(i2cMutex, timeout) != pdPASS)
    {
        return false; // Не удалось получить доступ к I2C
    }

    currentTrans = trans;
    xSemaphoreTake(completionSem, 0); // Сбрасываем семафор завершения

    // Проверка флага BUSY перед началом транзакции
    if (hi2c->ISR & I2C_ISR_BUSY_M)
    {
        I2C_RecoverBus();
        xSemaphoreGive(completionSem);
    }
    else
    {
        I2C_StartTransaction();
    }

    // Ожидаем завершения транзакции
    bool result = false;
    if (xSemaphoreTake(completionSem, timeout) == pdPASS)
    {
        if (result && (currentTrans->opMode != I2C_OP_WRITE_ONLY))
        {
            memcpy(currentTrans->readData, dma_rx_buffer, currentTrans->readDataLen);
        }
    }
    else
    {
        // Таймаут - прерываем транзакцию
        hi2c->CR2 |= I2C_CR2_STOP_M;
        I2C_RecoverBus();
        result = false;
    }

    currentTrans = NULL;
    xSemaphoreGive(i2cMutex); // Освобождаем мьютекс

    return result;
}

void Drv_I2C_Master_Off()
{
    hi2c->CR1 |= I2C_CR1_PE_M;
}

void Drv_I2C_Master_On()
{
    hi2c->CR1 &= ~I2C_CR1_PE_M;
}

// Запуск транзакции
static void I2C_StartTransaction(void)
{
    i2cState.cmdPos = 0;
    i2cState.dataPos = 0;
    i2cState.isReading = false;
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
            break;

        case I2C_OP_READ_ONLY:
            // Для операций чтения используем DMA
        	I2C_StartDmaReceive(dma_rx_buffer, currentTrans->readDataLen);
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
    hi2c->CR1 &= ~I2C_CR1_PE_M;
    hi2c->CR1 |= I2C_CR1_PE_M;

    hi2c->ICR = I2C_ICR_OVRCF_M | I2C_ICR_ARLOCF_M |
                I2C_ICR_BERRCF_M | I2C_ICR_STOPCF_M |
                I2C_ICR_NACKCF_M | I2C_ICR_ADDRCF_M;
}


// Обработчик прерываний DMA
void Drv_I2C_Master_DMA_IRQ_Handler(BaseType_t *pxHigherPriorityTaskWoken)
{
    if (DMA_CONFIG->CONFIG_STATUS & (1 << (DMA_STATUS_CHANNEL_IRQ_S + I2C_DMA_RX_CHANNEL)))
    {
        DMA_CONFIG->CONFIG_STATUS |= (1 << I2C_DMA_RX_CHANNEL);

        // Выключаем DMA прием в I2C
        hi2c->CR1 &= ~I2C_CR1_RXDMAEN_M;

        if (currentTrans)
        {
            xSemaphoreGiveFromISR(completionSem, pxHigherPriorityTaskWoken);
        }
    }
}

// Обработчик прерываний I2C
void Drv_I2C_Master_IRQ_Handler(BaseType_t *pxHigherPriorityTaskWoken)
{
    if (currentTrans == NULL) return;

    uint32_t isr = hi2c->ISR;

    if ((isr & (I2C_ISR_NACKF_M | I2C_ISR_BERR_M | I2C_ISR_ARLO_M | I2C_ISR_OVR_M)) ||
        (xTaskGetTickCountFromISR() - i2cState.startTime) > pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS))
    {
        hi2c->CR2 |= I2C_CR2_STOP_M;
        hi2c->ICR = I2C_ICR_NACKCF_M | I2C_ICR_BERRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_OVRCF_M;
        I2C_RecoverBus();
        //xSemaphoreGiveFromISR(completionSem, pxHigherPriorityTaskWoken);
        return;
    }

	switch (currentTrans->opMode)
	{
		case I2C_OP_WRITE_THEN_READ:
			if ((isr & I2C_ISR_TXIS_M) && (i2cState.cmdPos < currentTrans->writeDataLen) && !i2cState.isReading)
			{
				hi2c->TXDR = currentTrans->writeData[i2cState.cmdPos++];
			}

			if ((isr & I2C_ISR_TC_M) && (i2cState.cmdPos == currentTrans->writeDataLen) && !i2cState.isReading)
			{
				hi2c->CR2 &= ~(I2C_CR2_NBYTES_M | I2C_CR2_RD_WRN_M | I2C_CR2_AUTOEND_M);
				i2cState.isReading = true;

				// Запускаем DMA прием
				I2C_StartDmaReceive(dma_rx_buffer, currentTrans->readDataLen);
				hi2c->CR2 = (currentTrans->devAddr << 1) |
							I2C_CR2_NBYTES(currentTrans->readDataLen) |
							I2C_CR2_START_M |
							I2C_CR2_RD_WRN_M |
							I2C_CR2_AUTOEND_M;
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

			if ((isr & I2C_ISR_TC_M))
			{
				if (i2cState.cmdPos == currentTrans->writeDataLen)
				{
					hi2c->CR2 |= I2C_CR2_STOP_M;
					xSemaphoreGiveFromISR(completionSem, pxHigherPriorityTaskWoken);
				}
			}
			break;

		case I2C_OP_READ_ONLY:
			// Для READ_ONLY все обрабатывается в DMA, здесь ничего не делаем
			break;
	}
}
