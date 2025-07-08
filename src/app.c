/**
 * @file app.c
 * @brief Реализация модуля приложения
 * @author Evgenii Shkliaev
 * @date 18-06-2025
 */

#include "app.h"
#include "uart_to_i2c_protocol.h"


#define UART_BAUDRATE 9600

static void vAppTask(void *pvParameters);


bool App_Init(void)
{
	/* BSP Инициализация контролера управления LED*/
	if(!Bsp_LedC_Init_If())
    {
		return false;
    }

	 /* BSP Инициализация I2C */
	Drv_I2C_Master_Init(I2C_MASTER_SPEED_100kHz);

    /* BSP Инициализация Uart */
    if(!Bsp_UartTransport_Init_If(UART_BAUDRATE))
    {
    	return false;
    }

	if(xTaskCreate(vAppTask, "vAppTask", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 1 , NULL) != pdPASS)
	{
		return false;
	}

	return true;
}



void vAppTask(void *pvParameters)
{
	BSP_UartTransportMessage_t uartTxMsg;
	BSP_UartTransportMessage_t uartRxMsg;

    Bsp_LedC_SetLedMode_If(BSP_LEDC_LED_VD1, BSP_LEDC_MODE_ON);
    vTaskDelay(pdMS_TO_TICKS(500));
    Bsp_LedC_SetLedMode_If(BSP_LEDC_LED_VD1, BSP_LEDC_MODE_OFF);
    Bsp_LedC_SetLedMode_If(BSP_LEDC_LED_VD2, BSP_LEDC_MODE_1);

    while (1)
    {
        if (Bsp_UartTransportReceive_If(&uartRxMsg))
        {
            if (UART_Protocol_HandleCommand(&uartRxMsg, &uartTxMsg))
            {
                Bsp_UartTransportTransmit_If(&uartTxMsg);
                Bsp_LedC_SetLedMode_If(BSP_LEDC_LED_VD1, BSP_LEDC_MODE_ON);
            }
            Bsp_LedC_SetLedMode_If(BSP_LEDC_LED_VD1, BSP_LEDC_MODE_OFF);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

