
//#include "riscv_csr_encoding.h"
#include "scr1_csr_encoding.h"

#include "mik32_memory_map.h"
#include "power_manager.h"
#include "wakeup.h"
#include "gpio.h"
#include "pad_config.h"
#include "epic.h"
#include "uart.h"
#include "timer32.h"

#include "csr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#include "app.h"
#include "ports_driver.h"
#include "BspUartTransport_If.h"
#include "i2c_driver_master.h"
#include "wdt_driver.h"

extern unsigned long __TEXT_START__;


/*============================ RTOS Tasks ============================*/
void vTaskCheckHeap(void *pvParameters);
void WDT_Task(void *pvParameters);
/*============================ RTOS Tasks END ============================*/

/*============================ System Initialization ============================*/
static void SystemClock_Config(void);

/*============================ System Initialization END ============================*/

/*============================ Functions ============================*/
static void Error_handler(void);

/*============================ Functions END ============================*/


void main()

{
	// interrupt vector init
	write_csr(mtvec, &__TEXT_START__);
	SystemClock_Config();

    /* Создаем задачу watchdog */
    if(xTaskCreate(WDT_Task, "WDT", configMINIMAL_STACK_SIZE/2, NULL, configMAX_PRIORITIES - 1, NULL) != pdPASS)
    {
    	Error_handler();
    }

    /* Создаем задачу монитора остатка HEAP */
    if(xTaskCreate(vTaskCheckHeap, "HeapMonitor", configMINIMAL_STACK_SIZE/2, NULL, tskIDLE_PRIORITY, NULL) != pdPASS)
    {
    	Error_handler();
    }

	/* BSP Портов включения питания и наличия*/
	if(!Bsp_Ports_Init_If())
    {
    	Error_handler();
    }

    /* BSP Инициализация I2C */
    Drv_I2C_Master_Init();

    /* BSP Инициализация Uart */
    if(Bsp_UartTransport_Init_If(9600) == false)
    {
    	Error_handler();
    }

    /* Создаем задачу Application */
    if(!App_Init())
    {
    	Error_handler();
    }

    if(!WDT_Init())
    {
    	Error_handler();
    }
    WDT_Reset();

    // Запуск планировщика FreeRTOS
    vTaskStartScheduler();

    Error_handler();
}


/*============================ RTOS Tasks ============================*/
void WDT_Task(void *pvParameters)
{
    while (1)
    {
    	//WDT_Reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskCheckHeap(void *pvParameters)
{
	volatile size_t freeHeap = 0;
	while (1)
    {
        freeHeap = xPortGetFreeHeapSize();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Проверка каждую секунду
    }
}


/*============================ RTOS Tasks END ============================*/

static void SystemClock_Config(void)
{
    /*Включение тактирования модулей */
	PM->CLK_APB_M_SET =   PM_CLOCK_APB_M_PM_M
						| PM_CLOCK_APB_M_PAD_CONFIG_M
						| PM_CLOCK_APB_M_WU_M
						| PM_CLOCK_APB_M_EPIC_M;

	/* Выбор источника тактирования RTC в составе ядра */

	PM->CPU_RTC_CLK_MUX = PM_CPU_RTC_CLK_MUX_LSI32K_M;

	/* Настройка источника тактирования системы AHB_MUX */
	PM->AHB_CLK_MUX = PM_AHB_FORCE_MUX_UNFIXED | PM_AHB_CLK_MUX_OSC32M_M;

	/* Управления тактированием батарейного домена*/
	WU->CLOCKS_BU |= WU_CLOCKS_BU_RTC_CLK_MUX_LSI32K_M;
	WU->CLOCKS_BU |= WU_CLOCKS_BU_ADJ_LSI32K(8); //Поправочный коэффициент LSI32

	/* Управления тактированием системного домена */
	WU->CLOCKS_SYS |= WU_CLOCKS_SYS_ADJ_HSI32M(128); //Поправочный коэффициент HSI32M
}



/*============================ Errors handlers ============================*/
static void Error_handler(void)
{
	PM->CLK_APB_M_SET &= ~PM_CLOCK_APB_M_EPIC_M;
	do {} while (1);
}



/*============================ IRQ Handlers ============================*/
void ext_trap_handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (EPIC->RAW_STATUS & EPIC_LINE_M(EPIC_LINE_I2C_0_S))
    {
    	Drv_I2C_Master_IRQ_Handler(&xHigherPriorityTaskWoken);
        EPIC->CLEAR = (0b1) << EPIC_I2C_0_INDEX;
    }

    if (EPIC->RAW_STATUS & EPIC_LINE_M(EPIC_LINE_I2C_1_S))
    {
    	//Drv_I2C_Master_IRQ_Handler(&xHigherPriorityTaskWoken);
        EPIC->CLEAR = (0b1) << EPIC_I2C_1_INDEX;
    }

    if (EPIC->RAW_STATUS & EPIC_LINE_M(EPIC_LINE_UART_0_S))
    {
    	Bsp_UartTransport_Uart_IRQHandler_If(&xHigherPriorityTaskWoken);
    	EPIC->CLEAR = (0b1) << EPIC_UART_0_INDEX;
    }

    if(EPIC->RAW_STATUS & EPIC_LINE_M(EPIC_LINE_DMA_S))
    {
    	Bsp_UartTransport_DMA_IRQHandler_If();
    	EPIC->CLEAR = (0b1) << EPIC_DMA_INDEX;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
