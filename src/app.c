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


static void vAppTask(void *pvParameters);

bool App_Init(void)
{
    if(xTaskCreate(vAppTask, "vAppTask", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 1 , NULL) != pdPASS)
    {
        return false;
    }

    /* Инициализация протокола UART-I2C */
    Uart_to_i2c_Prot_Init(100);

    return true;
}


void vAppTask(void *pvParameters)
{
    BSP_UartTransportMessage_t uartTxMsg;
    BSP_UartTransportMessage_t uartRxMsg;

    while (1)
    {
		if (Bsp_UartTransportReceive_If(&uartRxMsg))
		{
			Uart_to_i2c_Prot_HandleCommand(&uartRxMsg, &uartTxMsg); // Блокирующая функция
			Bsp_UartTransportTransmit_If(&uartTxMsg);
		}

        //WDT_Reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



