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
 * 			[I2C_CMD]   		uint16_t (LSB-First)	- комманда слейву по I2C
 * 			[I2C_WRITE_LEN] 	uint8_t 				- длина данных для записи
 * 			[I2C_WRITE_DATA]	uint8_t 				- данные для записи
 *
 *
 * Ответ CMD_I2C_WRITE: [PackedID][CMD][STATUS]
 *
 *
 * Команда CMD_I2C_WRITE_THEN_READ:
 * [DATA] ->
 * 			[SA] 		uint8_t 				- адрес устройства
 * 			[I2C_CMD] 	uint16_t (LSB-First)	- комманда слейву по I2C
 * 			[READ_LEN] 	uint8_t 				- длина данных для чтения
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
}

static bool I2C_Transaction(uint8_t devAddr, uint16_t command, uint8_t cmdLen,
                          const uint8_t *writeData, uint8_t writeLen,
                          uint8_t *readData, uint8_t readLen)
{
    I2C_Master_Transaction_t trans = {0};
    bool result = false;

    // Если есть данные для записи, предполагаем, что первые 2 байта - это команда
    if (writeLen > 0 && writeData != NULL)
    {
        trans.writeData = (uint8_t *)writeData - 2; // Смещаем указатель на 2 байта назад
        trans.writeDataLen = writeLen + 2;
    }
    else
    {
        // Если данных нет, передаем только команду
        uint8_t cmd[2] = {command & 0xFF, (command >> 8) & 0xFF};
        trans.writeData = cmd;
        trans.writeDataLen = 2;
    }

    trans.devAddr = devAddr;
    trans.readData = readData;
    trans.readDataLen = readLen;
    trans.completionSem = xI2CSemaphore;

    // Определяем режим операции
    if (writeLen > 0 && readLen > 0) {
        trans.opMode = I2C_OP_WRITE_THEN_READ;
    }
    else if (writeLen > 0) {
        trans.opMode = I2C_OP_WRITE_ONLY;
    }
    else if (readLen > 0) {
        trans.opMode = I2C_OP_READ_ONLY;
    }
    else {
        trans.opMode = I2C_OP_WRITE_ONLY;
    }

    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        result = Drv_I2C_Master_SendTransaction(&trans, pdMS_TO_TICKS(100));
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

    switch (cmd)
    {
		case CMD_I2C_WRITE:
			// Проверяем минимальную длину: PacketID + CMD + SA + I2C_CMD(2) + WRITE_LEN = 7 байт
			if (rx_msg->payload_len < 6) return false;

			{
				uint8_t devAddr = rx_msg->payload[2];
				uint16_t i2cCmd = rx_msg->payload[3] | (rx_msg->payload[4] << 8);
				uint8_t writeLen = rx_msg->payload[5];

				// Проверяем что длина данных соответствует заявленной
				if (rx_msg->payload_len != 6 + writeLen) return false;

				// Выполняем запись
				bool result = I2C_Transaction(devAddr, i2cCmd, 2,
											&rx_msg->payload[6], writeLen,
											NULL, 0);

				tx_msg->payload[2] = result ? STATUS_OK : STATUS_ERROR;
				tx_msg->payload_len = 3;
				return true;
			}

		case CMD_I2C_WRITE_THEN_READ:
			// Проверяем длину: PacketID + CMD + SA + I2C_CMD(2) + READ_LEN = 6 байт
			if (rx_msg->payload_len != 6) return false;

			{
				uint8_t devAddr = rx_msg->payload[2];
				uint16_t i2cCmd = rx_msg->payload[3] | (rx_msg->payload[4] << 8);
				uint8_t readLen = rx_msg->payload[5];

				// Выполняем запись+чтение
				tx_msg->payload[2] = STATUS_OK; // Предварительно ставим OK
				bool result = I2C_Transaction(devAddr, i2cCmd, 2,
											NULL, 0,
											&tx_msg->payload[3], readLen);

				if (result)
				{
					tx_msg->payload[2] = STATUS_OK;
					tx_msg->payload_len = 3 + readLen;
				}
				else
				{
					tx_msg->payload[2] = STATUS_ERROR;
					tx_msg->payload_len = 3;
				}
				return true;
			}

        case CMD_GPIO_READ:
            // Проверяем длину данных: PacketID + CMD + PIN = 3 байта
            if (rx_msg->payload_len != 3) return false;
            {
                uint8_t pin = rx_msg->payload[2];
                bool pin_state = false;

                switch (pin) {
                    case 0: // Bsp_PrstPort
                        pin_state = Bsp_PrstPort_Read_If();
                        break;
                    case 1: // Bsp_VD2
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return false;
                    case 2: // Bsp_PwrOnPort
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return false;
                    default:
                        tx_msg->payload[2] = STATUS_ERROR;
                        tx_msg->payload_len = 3;
                        return false;
                }

                tx_msg->payload[2] = pin_state ? 1 : 0;
                tx_msg->payload_len = 3;
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
}

