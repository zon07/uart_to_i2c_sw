#include "uart_to_i2c_protocol.h"
#include "ports_driver.h"
#include "i2c_driver_master.h"
#include <string.h>
#include <stdbool.h>

/*
 * Запрос [PackedID][CMD][DATA]
 *
 * Команда CMD_I2C_WRITE:
 * [DATA] ->
 * 			[SA]  				uint8_t					- адрес устройства
 * 			[I2C_DATA_LEN] 	    uint8_t 				- длина данных для записи
 * 			[I2C_CMD]   		uint16_t (LSB-First)	- комманда слейву по I2C
 * 			[I2C_WRITE_DATA]	uint8_t 				- данные для записи
 *
 *
 * Ответ CMD_I2C_WRITE: [PackedID][CMD][STATUS]
 *
 *
 * Команда CMD_I2C_WRITE_THEN_READ:
 * [DATA] ->
 * 			[SA] 		       uint8_t 				- адрес устройства
 * 			[I2C_DATA_LEN] 	   uint8_t 				- длина данных для чтения
 * 			[I2C_CMD] 	       uint16_t (LSB-First)	- комманда слейву по I2C
 *
 * Ответ CMD_I2C_WRITE: [PackedID][CMD][STATUS][I2C_DATA]
 *
 *
 * Команда CMD_GPIO_READ:
 * [DATA] ->
 * 			[PIN]  uint8_t - номер пина
 *
 * Ответ CMD_GPIO_READ: [PackedID][CMD][STATUS][PIN_STATE]
 *
 *
 * Команда CMD_GPIO_WRITE:
 * [DATA] ->
 * 			[PIN] 	 uint8_t - номер пина
 * 			[STATE]  uint8_t - вкл/выкл
 *
 * Ответ CMD_GPIO_WRITE: [PackedID][CMD][STATUS]
 *
 *
 * Команда CMD_PING:
 * [DATA] -> 0
 *
 * Ответ CMD_PING: [PackedID][CMD][STATUS]
 *
 * */

static UART_Protocol_Context_t ctx;
// Статический семафор для I2C транзакций
static SemaphoreHandle_t xI2CSemaphore = NULL;

// Инициализация протокола
void UART_Protocol_Init(uint32_t i2c_timeout_ms)
{
    if (xI2CSemaphore == NULL)
    {
        xI2CSemaphore = xSemaphoreCreateBinary();
        xSemaphoreGive(xI2CSemaphore);
    }
    ctx.i2c_timeout_ms = i2c_timeout_ms;
}

static bool I2C_Transaction(uint8_t devAddr, const uint8_t *writeData, uint8_t writeLen, uint8_t *readData, uint8_t readLen)
{
    I2C_Master_Transaction_t trans = {0};
    bool result = false;

    // Настройка адреса и семафора
    trans.devAddr = devAddr;
    trans.completionSem = xI2CSemaphore;

    // Условная "команда" — это просто первые 2 байта в буфере writeData

    trans.writeData = (uint8_t *)writeData;  // Безопасное приведение типа
    trans.writeDataLen = writeLen;

    // Настройка чтения (если нужно)
    trans.readData = readData;
    trans.readDataLen = readLen;

    // Определение режима операции
    if (writeLen > 0 && readLen > 0)
    {
        trans.opMode = I2C_OP_WRITE_THEN_READ;
    }
    else if (writeLen > 0)
    {
        trans.opMode = I2C_OP_WRITE_ONLY;
    }
    else if (readLen > 0)
    {
        trans.opMode = I2C_OP_READ_ONLY;
    }
    else
    {
        return false;  // Ничего не делаем
    }

    // Захват семафора и выполнение транзакции
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(100)))
    {
        result = Drv_I2C_Master_SendTransaction(&trans, pdMS_TO_TICKS(ctx.i2c_timeout_ms));
        xSemaphoreGive(xI2CSemaphore);
    }

    return result;
}

bool UART_Protocol_HandleCommand(const BSP_UartTransportMessage_t* rx_msg, BSP_UartTransportMessage_t* tx_msg)
{
    if (rx_msg->payload_len < 2)
    {
        return false;
    }

    uint8_t packet_id = rx_msg->payload[0];
    uint8_t cmd = rx_msg->payload[1];

    tx_msg->payload[0] = packet_id;
    tx_msg->payload[1] = cmd;

    bool result;
    switch (cmd)
    {
		case CMD_I2C_WRITE:
		    // Проверяем длину: PacketID(1) + CMD(1) + SA(1) + I2C_DATA_LEN(1) + I2C_CMD(2) + данные(N)
		    // Минимальная длина: 6 байт (без данных), полная длина: 6 + N
		    if (rx_msg->payload_len < 6)
		        return false;

		    // Передаем: [SA][I2C_CMD][DATA]
		    // Структура данных в rx_msg->payload:
		    // [0] = PacketID
		    // [1] = CMD
		    // [2] = SA (адрес устройства)
		    // [3] = I2C_DATA_LEN (N)
		    // [4-5] = I2C_CMD (2 байта)
		    // [6..] = данные (N байт)
		    result = I2C_Transaction(
		        rx_msg->payload[2],       // devAddr (SA)
		        &rx_msg->payload[4],      // writeData (I2C_CMD + данные)
		        2 + rx_msg->payload[3],   // writeLen (2 байта команды + N байт данных)
		        NULL,                     // readData
		        0                         // readLen
		    );

		    tx_msg->payload[2] = result ? STATUS_OK : STATUS_ERROR;
		    tx_msg->payload_len = 3;
		    return result;

		case CMD_I2C_WRITE_THEN_READ:
		    // Проверяем длину: PacketID(1) + CMD(1) + SA(1) + I2C_DATA_LEN(1) + I2C_CMD(2)
		    // Для WRITE_THEN_READ данные не передаются, только длина чтения (I2C_DATA_LEN = READ_LEN)
		    if (rx_msg->payload_len != 6)
		        return false;

		    // Передаем: [SA][I2C_CMD], затем читаем READ_LEN байт
		    // I2C_DATA_LEN (rx_msg->payload[3]) содержит длину данных для чтения
		    result = I2C_Transaction(
		        rx_msg->payload[2],       // devAddr (SA)
		        &rx_msg->payload[4],      // writeData (I2C_CMD)
		        2,                        // writeLen (только 2 байта команды)
		        &tx_msg->payload[3],      // readData (куда писать результат)
		        rx_msg->payload[3]        // readLen (I2C_DATA_LEN)
		    );

		    tx_msg->payload[2] = result ? STATUS_OK : STATUS_ERROR;
		    tx_msg->payload_len = result ? (3 + rx_msg->payload[3]) : 3;
		    return result;

        case CMD_GPIO_READ:
            // Проверяем длину данных: PacketID + CMD + PIN = 3 байта
            if (rx_msg->payload_len != 3) return false;
            {
                uint8_t pin = rx_msg->payload[2];
                bool pin_state = false;

                switch (pin) {
                    case 0: // Bsp_PrstPort
                        pin_state = Bsp_PrstPort_Read_If();
                        tx_msg->payload[2] = STATUS_OK;
                        tx_msg->payload[3] = pin_state ? 1 : 0;
                        tx_msg->payload_len = 4;
                        break;
                    case 1: // Bsp_VD2
                    	pin_state = Bsp_VD2_Read_If();
                        tx_msg->payload[2] = STATUS_OK;
                        tx_msg->payload[3] = pin_state ? 1 : 0;
                        tx_msg->payload_len = 4;
                        return false;
                    case 2: // Bsp_PwrOnPort
                    	pin_state = Bsp_PwrOnPort_Read_If();
                        tx_msg->payload[2] = STATUS_OK;
                        tx_msg->payload[3] = pin_state ? 1 : 0;
                        tx_msg->payload_len = 4;
                        return false;
                    default:
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return false;
                }

                return true;
            }

        case CMD_GPIO_WRITE:
            // Проверяем длину данных: PacketID + CMD + PIN + STATE = 4 байта
            if (rx_msg->payload_len != 4) return false;

            {
                uint8_t pin = rx_msg->payload[2];
                uint8_t state = rx_msg->payload[3];

                switch (pin) {
                    case 0: // Bsp_PrstPort - запись запрещена
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return true;
                    case 1: // Bsp_VD2
                        if (state) {
                            Bsp_VD2_On_If();
                        } else {
                            Bsp_VD2_Off_If();
                        }
                        break;
                    case 2: // Bsp_PwrOnPort
                        if (state) {
                            Bsp_PwrOnPort_On_If();
                        } else {
                            Bsp_PwrOnPort_Off_If();
                        }
                        break;
                    default:
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return true;
                }

                tx_msg->payload[2] = STATUS_OK;
                tx_msg->payload_len = 3;
                return true;
            }

        case CMD_PING:
            // Для PING не должно быть никаких данных (только PacketID + CMD)
            if (rx_msg->payload_len != 2) return false;

            tx_msg->payload[2] = STATUS_OK;
            tx_msg->payload_len = 3;
            return true;

        default:
            return false;
    }
    return false;
}

