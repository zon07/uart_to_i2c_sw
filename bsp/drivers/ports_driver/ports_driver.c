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


#define VD2_PIN						(3U)
#define VD2_GPIO					GPIO_1
#define VD2_PAD_CONF_CFG			PORT_1_CFG
#define VD2_PAD_CONF_DS 			PORT_1_DS
#define VD2_PAD_CONF_PUPD 			PORT_1_PUPD

#define PWR_ON_PIN					(3U)
#define PWR_ON_GPIO					GPIO_0
#define PWR_ON_PAD_CONF_CFG			PORT_0_CFG
#define PWR_ON_PAD_CONF_DS 			PORT_0_DS
#define PWR_ON_PAD_CONF_PUPD 		PORT_0_PUPD

#define PRSNT_N_PIN					(4U)
#define PRSNT_N_GPIO				GPIO_0
#define PRSNT_N_PAD_CONF_CFG		PORT_0_CFG
#define PRSNT_N_PAD_CONF_DS 		PORT_0_DS
#define PRSNT_N_PAD_CONF_PUPD 		PORT_0_PUPD

#define USER_BTN_N_PIN				(8U)
#define USER_BTN_N_GPIO				GPIO_0
#define USER_BTN_N_PAD_CONF_CFG		PORT_0_CFG
#define USER_BTN_N_PAD_CONF_DS 		PORT_0_DS
#define USER_BTN_N_PAD_CONF_PUPD 	PORT_0_PUPD



bool Bsp_Ports_Init_If(void)
{
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_0_M;
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_1_M;

	/*Настройка VD2*/
	PAD_CONFIG->VD2_PAD_CONF_CFG 	&= ~PAD_CONFIG_PIN_M(VD2_PIN);			// Функция GPIO
	PAD_CONFIG->VD2_PAD_CONF_DS 	&= ~PAD_CONFIG_PIN_M(VD2_PIN);
	PAD_CONFIG->VD2_PAD_CONF_DS     |= PAD_CONFIG_PIN(VD2_PIN, 0b11); 		// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
	PAD_CONFIG->VD2_PAD_CONF_PUPD   &= ~PAD_CONFIG_PIN_M(VD2_PIN); 			// Подтяжки не включаем

	VD2_GPIO->DIRECTION_OUT |= (1 << VD2_PIN);  					// Как выход
	VD2_GPIO->OUTPUT 		&= ~(1 << VD2_PIN);

	/*Настройка PWR_ON*/
	PAD_CONFIG->PWR_ON_PAD_CONF_CFG 	&= ~PAD_CONFIG_PIN_M(PWR_ON_PIN);		// Функция GPIO
	PAD_CONFIG->PRSNT_N_PAD_CONF_DS 	&= ~PAD_CONFIG_PIN_M(PWR_ON_PIN);
	PAD_CONFIG->PRSNT_N_PAD_CONF_DS     |= PAD_CONFIG_PIN(PWR_ON_PIN, 0b11); 	// 0b00 – 2мА, 0b01 – 4мА, (0b10, 0b11) – 8мА
	PAD_CONFIG->PRSNT_N_PAD_CONF_PUPD   &= ~PAD_CONFIG_PIN_M(PWR_ON_PIN); 		// Подтяжки не включаем

	PWR_ON_GPIO->DIRECTION_OUT 	|= (1 << PWR_ON_PIN);  				// Как выход
	PWR_ON_GPIO->OUTPUT 		&= ~(1 << PWR_ON_PIN);

	/*Настройка PRSNT_N*/
	PAD_CONFIG->PRSNT_N_PAD_CONF_CFG 	&= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN);		// Функция GPIO
	PAD_CONFIG->PRSNT_N_PAD_CONF_DS 	&= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN);
	PAD_CONFIG->PRSNT_N_PAD_CONF_PUPD   &= ~PAD_CONFIG_PIN_M(PRSNT_N_PIN); 		// Подтяжки не включаем

	PRSNT_N_GPIO->DIRECTION_IN 	|= (1 << PRSNT_N_PIN);    			// Как вход

	/*Настройка USER_BTN_N*/
	PAD_CONFIG->USER_BTN_N_PAD_CONF_CFG 	&= ~PAD_CONFIG_PIN_M(USER_BTN_N_PIN);	// Функция GPIO
	PAD_CONFIG->USER_BTN_N_PAD_CONF_DS 		&= ~PAD_CONFIG_PIN_M(USER_BTN_N_PIN);
	PAD_CONFIG->USER_BTN_N_PAD_CONF_PUPD 	&= ~PAD_CONFIG_PIN_M(USER_BTN_N_PIN); 	// Подтяжки не включаем

	USER_BTN_N_GPIO->DIRECTION_IN 	|= (1 << USER_BTN_N_PIN);    		// Как вход

	return true;
}

void Bsp_VD2_On_If(void)
{
	taskENTER_CRITICAL();
	VD2_GPIO->SET = (1 << VD2_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_VD2_Off_If(void)
{
	taskENTER_CRITICAL();
	VD2_GPIO->CLEAR = (1 << VD2_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_PwrOnPort_On_If(void)
{
	taskENTER_CRITICAL();
	PWR_ON_GPIO->SET = (1 << PWR_ON_PIN);
	taskEXIT_CRITICAL();
}


void Bsp_PwrOnPort_Off_If(void)
{
	taskENTER_CRITICAL();
	PWR_ON_GPIO->CLEAR  = (1 << PWR_ON_PIN);
	taskEXIT_CRITICAL();
}



bool Bsp_VD2_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (VD2_GPIO->STATE >> VD2_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}

bool Bsp_PwrOnPort_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (PWR_ON_GPIO->STATE >> PWR_ON_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}

bool Bsp_PrstPort_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (PRSNT_N_GPIO->STATE >> PRSNT_N_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}


bool Bsp_UserBtn_Read_If(void)
{
    bool state;
    taskENTER_CRITICAL();
    state = (USER_BTN_N_GPIO->STATE >> USER_BTN_N_PIN) & 0x1;
    taskEXIT_CRITICAL();
    return state;
}
