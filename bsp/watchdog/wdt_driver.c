/*
 * wdt_driver.c
 *
 *  Created on: 11 июн. 2025 г.
 *      Author: user
 */

#include "wdt_driver.h"

#define WDT_CLOCK32M_VALUE 			(32000000U)
#define WDT_CLOCK32K_VALUE 			(32768U)
#define WDT_PRELOAD_MAX 			(0xFFF)

// Числовые значения предделителей
#define WDT_PRESCALER_VAL_1      	(1U)
#define WDT_PRESCALER_VAL_2      	(2U)
#define WDT_PRESCALER_VAL_4      	(4U)
#define WDT_PRESCALER_VAL_16     	(16U)
#define WDT_PRESCALER_VAL_64     	(64U)
#define WDT_PRESCALER_VAL_256    	(256U)
#define WDT_PRESCALER_VAL_1024   	(1024U)
#define WDT_PRESCALER_VAL_4096   	(4096U)

/*Источник тактирования*/
#define WDT_CLK_SOURCE				PM_WDT_CLK_MUX_OSC32M_M

#if (WDT_CLK_SOURCE == PM_WDT_CLK_MUX_LSI32K_M || WDT_CLK_SOURCE == PM_WDT_CLK_MUX_OSC32K_M)
	#define WDT_CLOCK					WDT_CLOCK32K_VALUE
#elif (WDT_CLK_SOURCE == PM_WDT_CLK_MUX_HSI32M_M || WDT_CLK_SOURCE == PM_WDT_CLK_MUX_OSC32M_M)
	#define WDT_CLOCK					WDT_CLOCK32M_VALUE
#else
	#error "Неверное значение WDT_CLK_SOURCE"
#endif

/* Битовая маска для регистра*/
#define WDT_PRESCALER           	WDT_CON_PRESCALE_4096_M

#if WDT_PRESCALER == WDT_CON_PRESCALE_1_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_1
#elif WDT_PRESCALER == WDT_CON_PRESCALE_2_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_2
#elif WDT_PRESCALER == WDT_CON_PRESCALE_4_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_4
#elif WDT_PRESCALER == WDT_CON_PRESCALE_16_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_16
#elif WDT_PRESCALER == WDT_CON_PRESCALE_64_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_64
#elif WDT_PRESCALER == WDT_CON_PRESCALE_256_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_256
#elif WDT_PRESCALER == WDT_CON_PRESCALE_1024_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_64
#elif WDT_PRESCALER == WDT_CON_PRESCALE_4096_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_4096
#else
	WDT_PRESCALER == WDT_CON_PRESCALE_1_M
#endif


#define WDT_PERIOD_MS				(500U)
// Рассчет PRELOAD (сначала умножение, потом деление!)
#define WDT_PRELOAD_VALUE        	(WDT_PRELOAD_MAX - ((WDT_PERIOD_MS * WDT_CLOCK) / (WDT_PRESCALER_VAL * 1000U)))
#if WDT_PRELOAD_VALUE > WDT_PRELOAD_MAX
	#error "Значение PRELOAD для WDT слишком большое для выбраных значений"
#endif


static bool WDT_Start(void);


/**
 * @brief Инициализация watchdog-таймера.
 */
bool WDT_Init()
{
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_WDT_M;
    PM->WDT_CLK_MUX = WDT_CLK_SOURCE;

    volatile uint32_t preloadV = WDT_PRELOAD_VALUE;
    WDT->KEY = WDT_KEY_UNLOCK;
    WDT->CON = WDT_CON_PRELOAD(preloadV) | WDT_PRESCALER;
    return WDT_Start();
}

bool WDT_Start(void)
{
	WDT->KEY = WDT_KEY_UNLOCK;
    WDT->KEY = WDT_KEY_START;  // Запуск таймера

    uint32_t timeout = 32; //таймаут ожидания включения
    while (timeout--)
    {
        if (WDT->STA & WDT_STA_ENABLED_M)
        {
            return true;
        }
    }
    return false;
}


/**
 * @brief Сброс watchdog-таймера (перезагрузка счетчика).
 */
void WDT_Reset(void)
{
    WDT->KEY = WDT_KEY_UNLOCK;
    WDT->KEY = WDT_KEY_START;
}

bool WDT_CheckResetFlag()
{
	return WDT->STA &= WDT_STA_RST_FLAG_M;
}

