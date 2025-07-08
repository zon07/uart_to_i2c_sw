/*
 * led_gpio.c
 *
 *  Created on: 6 июн. 2025 г.
 *      Author: zon07
 */

#include <leds_controller/leds.h>
#include "mik32_memory_map.h"
#include "power_manager.h"
#include "gpio.h"
#include "pad_config.h"



/*========== GPIO ==========*/
#define GPIO_PIN_0      0U
#define GPIO_PIN_1      1U
#define GPIO_PIN_2      2U
#define GPIO_PIN_3      3U
#define GPIO_PIN_4      4U
#define GPIO_PIN_5      5U
#define GPIO_PIN_6      6U
#define GPIO_PIN_7      7U
#define GPIO_PIN_8      8U
#define GPIO_PIN_9      9U
#define GPIO_PIN_10     10U
#define GPIO_PIN_11     11U
#define GPIO_PIN_12     12U
#define GPIO_PIN_13     13U
#define GPIO_PIN_14     14U
#define GPIO_PIN_15     15U


/*========== DRIVER MODES ==========*/
#define DRIVE_2MA  		0b00
#define DRIVE_4MA  		0b01
#define DRIVE_8MA_OPT1  0b11
#define DRIVE_8MA_OPT2  0b10



/**
 * @struct LED_Config
 * @brief Структура конфигурации светодиода
 *
 * Содержит параметры для настройки светодиода на конкретном порту GPIO.
 */
typedef struct {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t active_state;  // 0 для активного LOW, 1 для активного HIGH
    struct {
		volatile uint32_t* cfg_reg;   // Указатель на регистр конфигурации порта
		volatile uint32_t* ds_reg;    // Указатель на регистр drive strength
		volatile uint32_t* pupd_reg;  // Указатель на регистр pull-up/pull-down
		uint32_t drive_strength;
    } pad;
} LED_Config;


static const LED_Config led_config[BSP_LED_COUNT] = {
    [BSP_LED_VD1] = {
        .port = GPIO_0,
        .pin = GPIO_PIN_3,
        .active_state = 1, // Активный HIGH
        .pad = {
            .cfg_reg = &PAD_CONFIG->PORT_0_CFG,
            .ds_reg = &PAD_CONFIG->PORT_0_DS,
            .pupd_reg = &PAD_CONFIG->PORT_0_PUPD,
            .drive_strength = DRIVE_8MA_OPT2
        }
    },
    [BSP_LED_VD2] = {
        .port = GPIO_1,
        .pin = GPIO_PIN_3,
        .active_state = 1, // Активный HIGH
        .pad = {
            .cfg_reg = &PAD_CONFIG->PORT_1_CFG,
            .ds_reg = &PAD_CONFIG->PORT_1_DS,
            .pupd_reg = &PAD_CONFIG->PORT_1_PUPD,
            .drive_strength = DRIVE_8MA_OPT2
        }
    }
};



void BSP_LED_Init()
{
	/* Включение тактирования периферии */
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_0_M;
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_1_M;

	/* Инициализация VD1 LED */
    {
	    const LED_Config* vd1 = &led_config[BSP_LED_VD1];

	    *vd1->pad.cfg_reg &= ~PAD_CONFIG_PIN_M(vd1->pin); 	// Режим GPIO
	    *vd1->pad.ds_reg = (*vd1->pad.ds_reg & ~PAD_CONFIG_PIN_M(vd1->pin)) |
	                      PAD_CONFIG_PIN(vd1->pin, vd1->pad.drive_strength);
	    *vd1->pad.pupd_reg &= ~PAD_CONFIG_PIN_M(vd1->pin);	// Без подтяжки

	    vd1->port->DIRECTION_OUT |= (1 << vd1->pin);
	    vd1->port->DIRECTION_IN  &= ~(1 << vd1->pin);

	    BSP_LED_Set(BSP_LED_VD1, BSP_LED_STATE_OFF);
    }

    /* Инициализация VD2 LED */
    {
        const LED_Config* vd2 = &led_config[BSP_LED_VD2];

        *vd2->pad.cfg_reg &= ~PAD_CONFIG_PIN_M(vd2->pin); 	// Режим GPIO
        *vd2->pad.ds_reg = (*vd2->pad.ds_reg & ~PAD_CONFIG_PIN_M(vd2->pin)) |
                             PAD_CONFIG_PIN(vd2->pin, vd2->pad.drive_strength);
        *vd2->pad.pupd_reg &= ~PAD_CONFIG_PIN_M(vd2->pin); // Без подтяжки

        vd2->port->DIRECTION_OUT |= (1 << vd2->pin);
        vd2->port->DIRECTION_IN  &= ~(1 << vd2->pin);

        BSP_LED_Set(BSP_LED_VD2, BSP_LED_STATE_OFF);
    }
}


void BSP_LED_Set(BSP_Leds led, BSP_LedsState state)
{
    if (led >= BSP_LED_COUNT) return;

    const LED_Config* cfg = &led_config[led];

    switch (state) {
        case BSP_LED_STATE_ON:
            cfg->port->OUTPUT = (cfg->port->OUTPUT & ~(1 << cfg->pin)) |
                               ((cfg->active_state ? 1 : 0) << cfg->pin);
            break;

        case BSP_LED_STATE_OFF:
            cfg->port->OUTPUT = (cfg->port->OUTPUT & ~(1 << cfg->pin)) |
                               ((cfg->active_state ? 0 : 1) << cfg->pin);
            break;

        case BSP_LED_STATE_TOGGLE:
            cfg->port->OUTPUT ^= (1 << cfg->pin);
            break;
    }
}


void BSP_LED_Toggle(BSP_Leds led)
{
	BSP_LED_Set(led, BSP_LED_STATE_TOGGLE);
}

