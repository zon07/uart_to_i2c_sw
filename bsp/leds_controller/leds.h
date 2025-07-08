/**
 * @file leds.h
 * @brief Заголовочный файл для управления светодиодами
 *
 * Этот файл содержит определения и функции для работы со светодиодами системы.
 *
 * @author Evgenii Shkliaev (zon07)
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <stdint.h>

/**
 * @enum BSP_Leds
 * @brief Перечисление светодиодов
 *
 * Определяет доступные светодиоды.
 */
typedef enum {
    BSP_LED_VD1 = 0,
	BSP_LED_VD2,
    BSP_LED_COUNT
} BSP_Leds;

/**
 * @enum BSP_LedsState
 * @brief Состояния светодиодов
 *
 * Перечисление возможных состояний светодиодов.
 */
typedef enum {
    BSP_LED_STATE_OFF = 0,
    BSP_LED_STATE_ON,
    BSP_LED_STATE_TOGGLE
} BSP_LedsState;

/**
 * @brief Инициализация светодиодов
 *
 * Функция инициализации всех светодиодов.
 */
void BSP_LED_Init(void);

/**
 * @brief Установка состояния светодиода
 * @param led  Светодиод, состояние которого нужно установить
 * @param state Состояние светодиода (включен/выключен/переключение)
 */
void BSP_LED_Set(BSP_Leds led, BSP_LedsState state);

/**
 * @brief Переключение светодиода
 * @param led Светодиод, который нужно переключить
 */
void BSP_LED_Toggle(BSP_Leds led);


#endif /* LEDS_H_ */
