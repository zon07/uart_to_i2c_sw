/*
 * ports_driver.c
 *
 *  Created on: 9 июл. 2025 г.
 *      Author: user
 */

#include "ports_driver.h"

#include "mik32_memory_map.h"
#include "power_manager.h"
#include "gpio.h"
#include "pad_config.h"

#include "FreeRTOS.h"
#include "task.h"


#define VD2_PIN			(3U)
#define PWR_ON_PIN		(3U)
#define PRSNT_N_PIN		(4U)



bool Bsp_Ports_Init_If(void)
{
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_0_M;
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_1_M;

	/*Настройка VD2*/
	PAD_CONFIG->PORT_1_CFG 	&= ~PAD_CONFIG_PIN_M(VD2_PIN);		// Функция GPIO
	PAD_CONFIG->PORT_1_DS 	&= ~PAD_CONFIG_PIN_M(VD2_PIN);
	PAD_CONFIG->PORT_1_DS   |= PAD_CONFIG_PIN(VD2_PIN, 0b11); 	// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
	PAD_CONFIG->PORT_1_PUPD &= ~PAD_CONFIG_PIN_M(VD2_PIN); 		// Подтяжки не включаем

	GPIO_1->DIRECTION_OUT |= (1 << VD2_PIN);  // Как выход
	GPIO_1->DIRECTION_IN &= ~(1 << VD2_PIN);  // Отключить вход
	GPIO_1->OUTPUT &= ~(1 << VD2_PIN);

	/*Настройка PWR_ON*/
	PAD_CONFIG->PORT_0_CFG 	&= ~PAD_CONFIG_PIN_M(PWR_ON_PIN);		// Функция GPIO
	PAD_CONFIG->PORT_0_DS 	&= ~PAD_CONFIG_PIN_M(PWR_ON_PIN);
	PAD_CONFIG->PORT_0_DS   |= PAD_CONFIG_PIN(PWR_ON_PIN, 0b11); 	// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
	PAD_CONFIG->PORT_0_PUPD &= ~PAD_CONFIG_PIN_M(PWR_ON_PIN); 		// Подтяжки не включаем

	GPIO_0->DIRECTION_OUT |= (1 << PWR_ON_PIN);  // Как выход
	GPIO_0->DIRECTION_IN &= ~(1 << PWR_ON_PIN);  // Отключить вход
	GPIO_0->OUTPUT &= ~(1 << PWR_ON_PIN);

	/*Настройка PRSNT_N*/
	PAD_CONFIG->PORT_0_CFG 	&= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN);		// Функция GPIO
	PAD_CONFIG->PORT_0_DS 	&= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN);
	PAD_CONFIG->PORT_0_PUPD &= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN); 		// Подтяжки не включаем

	GPIO_0->DIRECTION_IN |= (1 << PRSNT_N_PIN);    // Как вход
	GPIO_0->DIRECTION_OUT &= ~(1 << PRSNT_N_PIN);  // Отключить выход

	return true;
}

void Bsp_VD2_On_If(void)
{
	taskENTER_CRITICAL();
	GPIO_1->SET = (1 << VD2_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_VD2_Off_If(void)
{
	taskENTER_CRITICAL();
	GPIO_1->CLEAR = (1 << VD2_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_PwrOnPort_On_If(void)
{
	taskENTER_CRITICAL();
	GPIO_0->SET = (1 << PWR_ON_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_PwrOnPort_Off_If(void)
{
	taskENTER_CRITICAL();
	GPIO_0->CLEAR  = (1 << PWR_ON_PIN);
	taskEXIT_CRITICAL();
}


bool Bsp_PrstPort_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (GPIO_0->STATE >> PRSNT_N_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}

bool Bsp_VD2_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (GPIO_1->STATE >> VD2_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}


bool Bsp_PwrOnPort_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (GPIO_0->STATE >> PWR_ON_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}

