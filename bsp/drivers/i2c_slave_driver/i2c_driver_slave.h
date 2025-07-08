#ifndef I2C_DRIVER_SLAVE_H_
#define I2C_DRIVER_SLAVE_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Размеры буферов (должны быть определены пользователем)
#ifndef I2C_SLAVE_CMD_SIZE
#define I2C_SLAVE_CMD_SIZE 2
#endif

#ifndef I2C_SLAVE_RX_BUFFER_SIZE
#define I2C_SLAVE_RX_BUFFER_SIZE 128
#endif

#ifndef I2C_SLAVE_TX_BUFFER_SIZE
#define I2C_SLAVE_TX_BUFFER_SIZE 128
#endif

typedef enum {
    I2C_SLAVE_EVENT_CMD_RECEIVED,    	// Получена полная команда
    I2C_SLAVE_EVENT_READ_RQST,       	// Запрос на чтение
	I2C_SLAVE_EVENT_RXDATA_READY,		// Получены данные
    I2C_SLAVE_EVENT_STOP,   			// STOP condition получен
    I2C_SLAVE_EVENT_ERROR            	// Ошибка передачи
} I2C_SlaveEvent_t;

typedef enum {
	I2C_SLAVE_PHASE_IDLE,
    I2C_SLAVE_PHASE_CMD,
	I2C_SLAVE_PHASE_DATA_RX,
	I2C_SLAVE_PHASE_DATA_TX
} I2C_SlavePhase_t;


typedef struct {
    uint8_t cmd[I2C_SLAVE_CMD_SIZE];     			// Принятая команда
    uint8_t cmd_byte_counter;      					// Счетчик байт команды
    I2C_SlavePhase_t phase;         			    // Фаза
    bool is_read_operation;               			// Флаг операции чтения
    bool transfer_complete;               			// Флаг завершения передачи
    uint16_t rx_length;                   			// Длина принятых данных
    uint16_t expected_rx_length;					// Ожидаемая длина данных
    uint16_t tx_length;                   			// Длина данных для передачи
    uint16_t tx_buffer_pos ;                   		// Длина данных для передачи
    uint8_t rx_buffer[I2C_SLAVE_RX_BUFFER_SIZE]; 	// Статический буфер приема
    uint8_t tx_buffer[I2C_SLAVE_TX_BUFFER_SIZE]; 	// Статический буфер передачи
} I2C_SlaveContext_t;

typedef bool (*I2C_SlaveCallback_t)(I2C_SlaveEvent_t event, I2C_SlaveContext_t* context);

typedef struct{
    I2C_SlaveCallback_t callback;    	// Callback-функция
    SemaphoreHandle_t completion_sem;	// Семафор завершения передачи
    uint8_t slaveAddr;               	// 7-bit адрес устройства
} I2C_SlaveConfig_t;


void Drv_I2C_Slave_Init(I2C_SlaveConfig_t* config);
void Drv_I2C_Slave_Enable(void);
void Drv_I2C_Slave_Disable(void);
void Drv_I2C_Slave_IRQ_Handler(BaseType_t* pxHigherPriorityTaskWoken);
bool Drv_I2C_Slave_CheckBus(void);
void Drv_I2C_Slave_Enable_SA(void);
void Drv_I2C_Slave_Disable_SA(void);

#endif /* I2C_DRIVER_SLAVE_H_ */
