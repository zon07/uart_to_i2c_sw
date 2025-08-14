/**
 * @file app.c
 * @brief Реализация модуля приложения
 * @author Evgenii Shkliaev
 * @date 18-06-2025
 */

#include "app.h"
#include "uart_to_i2c_protocol.h"
#include "i2c_driver_master.h"
#include "wdt_driver.h"
#include "ports_driver.h"


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

    /* Инициализация протокола UART-I2C */
    UART_Protocol_Init(100);

    return true;
}


void vAppTask(void *pvParameters)
{
	if (WDT_CheckResetFlag())
	{
		Bsp_VD2_On_If();
	}

    TickType_t xLastWakeTime100ms = xTaskGetTickCount();
    TickType_t xLastWakeTime1000ms = xTaskGetTickCount();

    BSP_UartTransportMessage_t uartTxMsg;
    BSP_UartTransportMessage_t uartRxMsg;

    while (1)
    {
        TickType_t xNow = xTaskGetTickCount();

		if (Bsp_UartTransportReceive_If(&uartRxMsg))
		{
			UART_Protocol_HandleCommand(&uartRxMsg, &uartTxMsg); // Блокирующая функция
			Bsp_UartTransportTransmit_If(&uartTxMsg);
		}

        WDT_Reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



