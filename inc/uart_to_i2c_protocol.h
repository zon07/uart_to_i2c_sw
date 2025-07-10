#ifndef UART_TO_I2C_PROTOCOL_H
#define UART_TO_I2C_PROTOCOL_H

//#include "i2c_driver_master.h"
#include "BspUartTransport_If.h"
#include <stdbool.h>

// Команды протокола
typedef enum {
    CMD_I2C_WRITE = 0x01,
    CMD_I2C_WRITE_THEN_READ = 0x02,
    CMD_GPIO_READ = 0x10,
    CMD_GPIO_WRITE = 0x11,
    CMD_PING = 0xFF
} UART_Protocol_CMD_t;

// Статусы ответов
typedef enum {
    STATUS_OK = 0x00,
    STATUS_ERROR = 0x01,
    STATUS_CRC_ERR = 0x02
} UART_Protocol_Status_t;

// Контекст протокола
typedef struct {
    uint32_t i2c_timeout_ms;
} UART_Protocol_Context_t;

// Инициализация протокола
void UART_Protocol_Init(uint32_t i2c_timeout_ms);

// Обработчик входящих команд
bool UART_Protocol_HandleCommand(const BSP_UartTransportMessage_t* rx_msg, BSP_UartTransportMessage_t* tx_msg);

#endif // UART_TO_I2C_PROTOCOL_H
