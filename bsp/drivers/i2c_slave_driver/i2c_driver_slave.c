/*
 * i2c_driver_slave.c
 *
 *  Created on: 30 июн. 2025 г.
 *      Author: Evgenii Shkliaev (zon07)
 */

#include <string.h>

#include "i2c_driver_slave.h"

#include "mik32_memory_map.h"
#include "power_manager.h"
#include "pad_config.h"
#include "i2c.h"
#include "epic.h"

/* Выбор канала 0 или 1*/
#define DRIVER_I2C_SLAVE_CHANNEL     (0U)

// Настройки I2C (аппаратные)
#if (DRIVER_I2C_SLAVE_CHANNEL == 0)
	#define I2C_INSTANCE I2C_0
    #define DRIVER_I2C_SLAVE_EPIC_LINE        (EPIC_LINE_I2C_0_S)
    #define DRIVER_I2C_SLAVE_PM_CLOCK_APB_P   (PM_CLOCK_APB_P_I2C_0_M)

    #define DRIVER_I2C_SLAVE_PORT_CFG         (PAD_CONFIG->PORT_0_CFG)
    #define DRIVER_I2C_SLAVE_PORT_DS          (PAD_CONFIG->PORT_0_DS)
    #define DRIVER_I2C_SLAVE_PORT_PUPD        (PAD_CONFIG->PORT_0_PUPD)

    #define DRIVER_I2C_SLAVE_SDA_PIN          (9)
    #define DRIVER_I2C_SLAVE_SCL_PIN          (10)
#else
	#define I2C_INSTANCE I2C_1
    #define DRIVER_I2C_SLAVE_EPIC_LINE        (EPIC_LINE_I2C_1_S)
    #define DRIVER_I2C_SLAVE_PM_CLOCK_APB_P   (PM_CLOCK_APB_P_I2C_1_M)

    #define DRIVER_I2C_SLAVE_PORT_CFG         (PAD_CONFIG->PORT_1_CFG)
    #define DRIVER_I2C_SLAVE_PORT_DS          (PAD_CONFIG->PORT_1_DS)
    #define DRIVER_I2C_SLAVE_PORT_PUPD        (PAD_CONFIG->PORT_1_PUPD)

    #define DRIVER_I2C_SLAVE_SDA_PIN          (12)
    #define DRIVER_I2C_SLAVE_SCL_PIN          (13)
#endif

#define I2C_SLAVE_BUFFER_SIZE 64
#if I2C_SLAVE_BUFFER_SIZE > 255
    #error "I2C_SLAVE_BUFFER_SIZE must be <= 255"
#endif


static I2C_TypeDef* hi2c = I2C_INSTANCE;
static I2C_SlaveConfig_t* slaveConfig = NULL;
static I2C_SlaveContext_t slaveCtx;


// Инициализация драйвера
void Drv_I2C_Slave_Init(I2C_SlaveConfig_t* config)
{
    if (!config || !config->callback) return;

    slaveConfig = config;

    // Инициализация контекста
    memset(&slaveCtx, 0, sizeof(I2C_SlaveContext_t));

    if (!config->completion_sem)
    {
        config->completion_sem = xSemaphoreCreateBinary();
    }

    // Аппаратная инициализация I2C
    PM->CLK_APB_P_SET = DRIVER_I2C_SLAVE_PM_CLOCK_APB_P;

    // Настройка пинов
    DRIVER_I2C_SLAVE_PORT_DS     &= ~PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SDA_PIN);
    DRIVER_I2C_SLAVE_PORT_DS      |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SDA_PIN, 0b11);
    DRIVER_I2C_SLAVE_PORT_PUPD   &= ~PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SDA_PIN);
    DRIVER_I2C_SLAVE_PORT_PUPD    |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SDA_PIN, 0b01);

    DRIVER_I2C_SLAVE_PORT_DS     &= ~PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SCL_PIN);
    DRIVER_I2C_SLAVE_PORT_DS      |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SCL_PIN, 0b11);
    DRIVER_I2C_SLAVE_PORT_PUPD   &= ~PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SCL_PIN);
    DRIVER_I2C_SLAVE_PORT_PUPD    |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SCL_PIN, 0b01);

    DRIVER_I2C_SLAVE_PORT_CFG &= ~(PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SDA_PIN) | PAD_CONFIG_PIN_M(DRIVER_I2C_SLAVE_SCL_PIN));
    DRIVER_I2C_SLAVE_PORT_CFG |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SDA_PIN, 0b01);
    DRIVER_I2C_SLAVE_PORT_CFG |= PAD_CONFIG_PIN(DRIVER_I2C_SLAVE_SCL_PIN, 0b01);


    // Выключаем модуль I2C перед настройкой
    hi2c->CR1 &= ~I2C_CR1_PE_M;

    // Настройка фильтров
    hi2c->CR1 &= ~I2C_CR1_ANFOFF_M;
    hi2c->CR1 |= (0 << I2C_CR1_ANFOFF_S);

    hi2c->CR1 &= ~I2C_CR1_DNF_M;
    hi2c->CR1 |= I2C_CR1_DNF(0);

    // Настройка временных параметров
    hi2c->TIMINGR = 0;
    hi2c->TIMINGR |= I2C_TIMINGR_PRESC(1);

    // Настройка растягивания CLK и аппаратный контроль передачи данных
    hi2c->CR1 &= ~(I2C_CR1_SBC_M | I2C_CR1_NOSTRETCH_M);
    hi2c->CR2 &= ~I2C_CR2_RELOAD_M | I2C_CR2_NACK_M;

    // Включаем SBC и RELOAD режим при адресации
    hi2c->CR1 |= I2C_CR1_SBC_M;
    hi2c->CR2 |= I2C_CR2_RELOAD_M;

    // Настройка адреса slave
    hi2c->CR1 &= ~I2C_CR1_GCEN_M;
    hi2c->OAR1 = 0;
    hi2c->OAR1 = (config->slaveAddr << 1) | I2C_OAR1_OA1EN_M;

    // Включение прерываний в EPIC
    EPIC->MASK_EDGE_SET = EPIC_LINE_M(DRIVER_I2C_SLAVE_EPIC_LINE);

}


void Drv_I2C_Slave_Enable(void)
{
    if (!slaveConfig) return;

    // Сброс состояния
    memset(&slaveCtx, 0, sizeof(I2C_SlaveContext_t));

    slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
    slaveCtx.cmd_byte_counter = 0;

    // Сброс флагов прерываний
    hi2c->ICR = I2C_ICR_OVRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_BERRCF_M |
                I2C_ICR_STOPCF_M | I2C_ICR_NACKCF_M | I2C_ICR_ADDRCF_M | I2C_ISR_STOPF_M;

    // Включение прерываний I2C
    hi2c->CR1 |= I2C_CR1_ERRIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TCIE_M |
                 I2C_CR1_TXIE_M | I2C_CR1_RXIE_M | I2C_CR1_ADDRIE_M | I2C_ISR_STOPF_M;

    // Включение модуля I2C
    hi2c->CR1 |= I2C_CR1_PE_M;
}

void Drv_I2C_Slave_Disable(void)
{
    if (!slaveConfig) return;

    // Отключение прерываний
    hi2c->CR1 &= ~(I2C_CR1_ERRIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TCIE_M |
                   I2C_CR1_TXIE_M | I2C_CR1_RXIE_M | I2C_CR1_ADDRIE_M | I2C_ISR_STOPF_M);

    // Отключение модуля I2C
    hi2c->CR1 &= ~I2C_CR1_PE_M;
}

bool Drv_I2C_Slave_CheckBus(void)
{
	return (hi2c->ISR & I2C_ISR_BUSY_M);
}

void Drv_I2C_Slave_Enable_SA(void)
{
	if(!(hi2c->OAR1 & I2C_OAR1_OA1EN_M))
	{
	    hi2c->OAR1 |= I2C_OAR1_OA1EN_M;
	}
}

void Drv_I2C_Slave_Disable_SA(void)
{
    hi2c->OAR1 &= ~I2C_OAR1_OA1EN_M;
}


static void HandleError(BaseType_t* pxHigherPriorityTaskWoken)
{
    slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
    slaveCtx.transfer_complete = true;
    if (slaveConfig->callback)
    {
        slaveConfig->callback(I2C_SLAVE_EVENT_ERROR, &slaveCtx);
    }
    if (slaveConfig->completion_sem)
    {
        xSemaphoreGiveFromISR(slaveConfig->completion_sem, pxHigherPriorityTaskWoken);
    }
    hi2c->ICR = I2C_ICR_NACKCF_M | I2C_ICR_BERRCF_M | I2C_ICR_ARLOCF_M | I2C_ICR_OVRCF_M;
}

// Обработчик прерываний I2C Slave
// [ADDR][W][CMD](!EVENT_CMD)[DATA]()[STOP](!EVENT_STOP)
// [ADDR][W][CMD](!EVENT_CMD)[ADDR][R](!EVENT_DATA_RQST)[DATA][STOP](!EVENT_STOP)

void Drv_I2C_Slave_IRQ_Handler(BaseType_t* pxHigherPriorityTaskWoken)
{
    uint32_t isr = hi2c->ISR;

    // Обработка ошибок
    if (isr & (I2C_ISR_NACKF_M | I2C_ISR_BERR_M | I2C_ISR_ARLO_M | I2C_ISR_OVR_M))
    {
        HandleError(pxHigherPriorityTaskWoken);
        return;
    }

    // Обработка совпадения адреса
    if (isr & I2C_ISR_ADDR_M)
    {
        slaveCtx.rx_length = 0;
        slaveCtx.tx_length = 0;
        slaveCtx.tx_buffer_pos = 0;
        slaveCtx.cmd_byte_counter = 0;
        slaveCtx.transfer_complete = false;
        slaveCtx.is_read_operation = (isr & I2C_ISR_DIR_M) ? 1 : 0;

        // Настройка приема одного байта с контролем ACK
        hi2c->CR2 = (hi2c->CR2 & ~(I2C_CR2_NBYTES_M | I2C_CR2_NACK_M)) | I2C_CR2_NBYTES(1);

        if(slaveCtx.is_read_operation)
        {
            slaveCtx.phase = I2C_SLAVE_PHASE_DATA_TX;
            if (slaveConfig->callback)
            {
                slaveConfig->callback(I2C_SLAVE_EVENT_READ_RQST, &slaveCtx);
            }
        }
        else
        {
            slaveCtx.phase = I2C_SLAVE_PHASE_CMD;
        }

        hi2c->ICR = I2C_ICR_ADDRCF_M;
        return;
    }

    // Обработка приема данных (запись)
    if ((isr & I2C_ISR_RXNE_M) && !slaveCtx.is_read_operation)
    {
        uint8_t data = hi2c->RXDR;

        if (slaveCtx.phase == I2C_SLAVE_PHASE_CMD)
        {
            if (slaveCtx.cmd_byte_counter < I2C_SLAVE_CMD_SIZE)
            {
                slaveCtx.cmd[slaveCtx.cmd_byte_counter++] = data;

                if (slaveCtx.cmd_byte_counter == I2C_SLAVE_CMD_SIZE)
                {
                    slaveCtx.phase = I2C_SLAVE_PHASE_DATA_RX;
                    if (slaveConfig->callback)
                    {
                        if(!slaveConfig->callback(I2C_SLAVE_EVENT_CMD_RECEIVED, &slaveCtx))
                        {
                            hi2c->CR2 |= I2C_CR2_NACK_M;
                            slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
                        }
                    }
                }

                // Готовимся к приему следующего байта
                hi2c->CR2 = (hi2c->CR2 & ~I2C_CR2_NBYTES_M) | I2C_CR2_NBYTES(1);
            }
        }
        else if (slaveCtx.phase == I2C_SLAVE_PHASE_DATA_RX)
        {
            if (slaveCtx.rx_length < I2C_SLAVE_RX_BUFFER_SIZE)
            {
                slaveCtx.rx_buffer[slaveCtx.rx_length++] = data;

                if (slaveConfig->callback && (slaveCtx.rx_length == slaveCtx.expected_rx_length))
                {
                    if (!slaveConfig->callback(I2C_SLAVE_EVENT_RXDATA_READY, &slaveCtx))
                    {
                        hi2c->CR2 |= I2C_CR2_NACK_M;
                        slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
                        return;
                    }
                }

                // Готовимся к приему следующего байта
                hi2c->CR2 = (hi2c->CR2 & ~I2C_CR2_NBYTES_M) | I2C_CR2_NBYTES(1);
            }
            else
            {
                hi2c->CR2 |= I2C_CR2_NACK_M;
                slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
            }
        }
        return;
    }

    // Обработка передачи данных (чтение)
    if ((isr & I2C_ISR_TXIS_M) && slaveCtx.is_read_operation &&
        (slaveCtx.phase == I2C_SLAVE_PHASE_DATA_TX))
    {
        if (slaveCtx.tx_buffer_pos < slaveCtx.tx_length)
        {
            hi2c->TXDR = slaveCtx.tx_buffer[slaveCtx.tx_buffer_pos++];
        }
        else
        {
            hi2c->TXDR = 0xFF;
        }

        // Готовимся к передаче следующего байта
        hi2c->CR2 = (hi2c->CR2 & ~I2C_CR2_NBYTES_M) | I2C_CR2_NBYTES(1);
        return;
    }

    // Обработка STOP condition
    if (isr & I2C_ISR_STOPF_M)
    {
        slaveCtx.phase = I2C_SLAVE_PHASE_IDLE;
        slaveCtx.transfer_complete = true;

        if (slaveConfig->callback)
        {
            slaveConfig->callback(I2C_SLAVE_EVENT_STOP, &slaveCtx);
        }

        if (slaveConfig->completion_sem)
        {
            xSemaphoreGiveFromISR(slaveConfig->completion_sem, pxHigherPriorityTaskWoken);
        }

        hi2c->ICR = I2C_ICR_STOPCF_M;
    }
}
