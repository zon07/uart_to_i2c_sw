/**
 * @file app.c
 * @brief Реализация модуля приложения
 * @author Evgenii Shkliaev
 * @date 18-06-2025
 */

#include "app.h"
#include "uart_to_i2c_protocol.h"
#include "i2c_driver_master.h"


#define SLAVE_ADDRESS 0x40    // Адрес слейва (7-битный)

#define SNS_COUNT_PLAN 					(5U)

#define COMMAND_GET_SW_VERSION 				(0x0002U)
#define COMMAND_GET_SW_VERSION_SIZE 		(3*3U)

#define COMMAND_GET_SNS_CNT 				(0x0003U)
#define COMMAND_GET_SNS_CNT_DATA_SIZE 		(3U)

#define COMMAND_GET_SNS_TYPES 				(0x1000U)
#define COMMAND_GET_SNS_TYPES_DATA_SIZE 	(9*SNS_COUNT_PLAN)


static SemaphoreHandle_t i2c_sem;

static void vAppTask(void *pvParameters);


bool App_Init(void)
{
    if(xTaskCreate(vAppTask, "vAppTask", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 1 , NULL) != pdPASS)
    {
        return false;
    }

    i2c_sem = xSemaphoreCreateBinary();

    return true;
}


void vAppTask(void *pvParameters)
{
    TickType_t xLastWakeTime100ms = xTaskGetTickCount();
    TickType_t xLastWakeTime1000ms = xTaskGetTickCount();

    BSP_UartTransportMessage_t uartTxMsg;
    BSP_UartTransportMessage_t uartRxMsg;

    uint16_t index = 0;
    uint16_t command = COMMAND_GET_SNS_TYPES;
    uint8_t cmd[2] = {command >> 8, command & 0xFF};

    uint8_t i2c_rxBuff[255] = {0};

    I2C_Master_Transaction_t trans = {
        .devAddr = SLAVE_ADDRESS,
        .opMode = I2C_OP_WRITE_THEN_READ,
        .writeData = cmd,
        .writeDataLen = 2,
        .readData = i2c_rxBuff,
        .readDataLen = COMMAND_GET_SNS_TYPES_DATA_SIZE,
        .completionSem = i2c_sem,
        .result = false
    };

    while (1)
    {
        TickType_t xNow = xTaskGetTickCount();

		if (Bsp_UartTransportReceive_If(&uartRxMsg))
		{
			if (UART_Protocol_HandleCommand(&uartRxMsg, &uartTxMsg))
			{
				Bsp_UartTransportTransmit_If(&uartTxMsg);
			}
		}


        // Обработка 100мс интервала
        if ((xTaskGetTickCount() - xLastWakeTime100ms) >= pdMS_TO_TICKS(100))
        {
        	xLastWakeTime100ms = xTaskGetTickCount();

        }

        // Обработка 1000мс интервала
        if ((xTaskGetTickCount() - xLastWakeTime1000ms) >= pdMS_TO_TICKS(1000))
        {
        	xLastWakeTime1000ms = xTaskGetTickCount();
            bool result = Drv_I2C_Master_SendTransaction(&trans, pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



