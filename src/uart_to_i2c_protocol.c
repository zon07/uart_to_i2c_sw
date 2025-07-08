#include "uart_to_i2c_protocol.h"
#include <string.h>
#include <stdbool.h>


static bool Handle_I2C_Write(uint8_t packet_id, const uint8_t* data, uint16_t len, BSP_UartTransportMessage_t* tx_msg);
static bool Handle_I2C_Read(uint8_t packet_id, const uint8_t* data, uint16_t len, BSP_UartTransportMessage_t* tx_msg);

bool UART_Protocol_HandleCommand(const BSP_UartTransportMessage_t* rx_msg, BSP_UartTransportMessage_t* tx_msg)
{
    if (rx_msg->payload_len < 2)
    {
    	return false; // Минимум PacketID + CMD
    }

    uint8_t packet_id = rx_msg->payload[0];
    uint8_t cmd = rx_msg->payload[1];
    const uint8_t* data = rx_msg->payload + 2;
    uint16_t data_len = rx_msg->payload_len - 2;

    tx_msg->payload[0] = packet_id; // Копируем PacketID в ответ

    switch (cmd)
    {
        case CMD_I2C_WRITE:
            tx_msg->payload[1] = CMD_I2C_WRITE | 0x80; // Ответная команда
            if (Handle_I2C_Write(packet_id, data, data_len, tx_msg))
            {
                tx_msg->payload[2] = STATUS_OK;
                tx_msg->payload_len = 3; // ID + CMD + STATUS
            }
            else
            {
                tx_msg->payload[2] = STATUS_ERROR;
                tx_msg->payload_len = 3;
            }
            return true;

        case CMD_I2C_READ:
            tx_msg->payload[1] = CMD_I2C_READ | 0x80;
            if (Handle_I2C_Read(packet_id, data, data_len, tx_msg))
            {
                tx_msg->payload[2] = STATUS_OK;
                // Данные уже добавлены в tx_msg в Handle_I2C_Read
            }
            else
            {
                tx_msg->payload[2] = STATUS_ERROR;
                tx_msg->payload_len = 3;
            }
            return true;

        case CMD_PING:
            tx_msg->payload[1] = CMD_PING;
            tx_msg->payload_len = 2;
            return true;

        default:
            return false;
    }
}

static bool Handle_I2C_Write(uint8_t packet_id, const uint8_t* data, uint16_t len, BSP_UartTransportMessage_t* tx_msg)
{
    if (len < 2) return false; // Минимум адрес + 1 байт данных

    uint8_t dev_addr = data[0];
    I2C_Master_Transaction_t trans ={
        .devAddr = dev_addr,
        .opMode = I2C_MASTER_OP_WRITE_ONLY,
        .command = (uint8_t*)(data + 1),
        .commandLen = len - 1,
        .data = NULL,
        .dataLen = 0,
        .completionSem = xSemaphoreCreateBinary(),
        .result = false
    };

    if (Drv_I2C_Master_SendTransaction(&trans, pdMS_TO_TICKS(100)))
    {
        return trans.result;
    }
    return false;
}

static bool Handle_I2C_Read(uint8_t packet_id, const uint8_t* data, uint16_t len, BSP_UartTransportMessage_t* tx_msg)
{
    if (len < 2) return false; // Минимум адрес + длина

    uint8_t dev_addr = data[0];
    uint8_t read_len = data[1];
    static uint8_t i2c_buffer[32]; // Буфер для чтения

    I2C_Master_Transaction_t trans = {
        .devAddr = dev_addr,
        .opMode = I2C_MASTER_OP_READ_ONLY,
        .command = NULL,
        .commandLen = 0,
        .data = i2c_buffer,
        .dataLen = read_len,
        .completionSem = xSemaphoreCreateBinary(),
        .result = false
    };

    if (Drv_I2C_Master_SendTransaction(&trans, pdMS_TO_TICKS(100)))
    {
        if (trans.result)
        {
            tx_msg->payload_len = 3 + read_len; // ID + CMD + STATUS + данные
            memcpy(tx_msg->payload + 3, i2c_buffer, read_len);
            return true;
        }
    }
    return false;
}
