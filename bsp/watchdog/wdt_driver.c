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
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_1024
#elif WDT_PRESCALER == WDT_CON_PRESCALE_4096_M
	#define WDT_PRESCALER_VAL       	WDT_PRESCALER_VAL_4096
#else
	WDT_PRESCALER == WDT_CON_PRESCALE_1_M
#endif


#define WDT_PERIOD_MS				(500U)
#define WDT_PRELOAD_VALUE        	(WDT_PRELOAD_MAX - ((WDT_PERIOD_MS * (WDT_CLOCK / 1000U)) / WDT_PRESCALER_VAL))
#if WDT_PRELOAD_VALUE > WDT_PRELOAD_MAX
	#error "Значение PRELOAD для WDT слишком большое для выбраных значений"
#endif


static bool WDT_Start(void);

/**
 * @brief  Запись слова в регистр CON (конфигурация WDT)
 * @param  hwdt указатель на структуру WDT_HandleTypeDef, которая содержит
 *                  информацию о конфигурации для модуля WDT.
 * @param  conValue 32-х битное слово
 */
static void WDT_Write_Word_To_CON(WDT_TypeDef* hwdt, uint32_t conValue)
{
	//Если позволить компилятору inlining, станет невозможно выделить функцию в отдельную секцию.
	intptr_t ptr = (intptr_t)(hwdt);
	/**
	 *  Попытки избежать использования отдельной переменной-указателя, такие как:
	 *  "sb a0,0x9C(%a0)\n\t"
	 *  : "m" (*(hwdt->Instance))
	 *  не работают из-за бага GCC (internal compiler error <...> riscv_print_operand_address)
	 * */
	asm volatile(
		"li a0,0x1E\n\t" //Store unlock byte somewhere
		"sb a0,0x9C(%0)\n\t" //Write UNLOCK byte into WDT KEY register
		"sw %1,0x84(%0)\n\t" //Write payload
		:
		: "r" (ptr), "r" (conValue)
		: "a0"
	);
}
/**
 * @brief  Запись байта в регистр KEY (пуск/остановка WDT)
 * @param  hwdt указатель на структуру WDT_HandleTypeDef, которая содержит
 *                  информацию о конфигурации для модуля WDT.
 * @param  key байт для записи ( @ref WDT_KEY_START или @ref WDT_KEY_STOP )
 */
static void WDT_Write_Byte_To_KEY(WDT_TypeDef* hwdt, uint8_t key)
{
	intptr_t ptr = (intptr_t)(hwdt);
	asm volatile(
		"li a0,0x1E\n\t" //Store unlock byte somewhere
		"sb a0,0x9C(%0)\n\t" //Write UNLOCK byte into WDT KEY register
		"sb %1,0x9C(%0)\n\t" //Write payload
		:
		: "r" (ptr), "r" (key)
		: "a0"
	);
}


/**
 * @brief Инициализация watchdog-таймера.
 */
bool WDT_Init()
{
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_WDT_M;
    PM->WDT_CLK_MUX = WDT_CLK_SOURCE;

    uint32_t conValue = WDT_CON_PRELOAD(WDT_PRELOAD_VALUE) | WDT_PRESCALER;

    WDT_Write_Byte_To_KEY(WDT, WDT_KEY_UNLOCK);
    WDT_Write_Word_To_CON(WDT, conValue);

    return WDT_Start();
}

bool WDT_Start(void)
{
	WDT_Write_Byte_To_KEY(WDT, WDT_KEY_UNLOCK);
	WDT_Write_Byte_To_KEY(WDT, WDT_KEY_START);

    uint32_t timeout = 0xFFFF; //таймаут ожидания включения
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
	WDT_Write_Byte_To_KEY(WDT, WDT_KEY_UNLOCK);
	WDT_Write_Byte_To_KEY(WDT, WDT_KEY_START);
}

bool WDT_CheckResetFlag()
{
	return WDT->STA &= WDT_STA_RST_FLAG_M;
}

