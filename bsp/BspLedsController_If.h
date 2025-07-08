/**
  * @file BspLedsController_If.h
  * @brief Интерфейс контроллера светодиодов (LED)
  * @author Evgenii Shkliaev (zon07)
  * @date 10-06-2025
 */

#ifndef INC_LEDSCONTROLLER_IF_H_
#define INC_LEDSCONTROLLER_IF_H_

#include <stdbool.h>

/**
 * @brief Идентификаторы светодиодов
 * @note Используется для обращения к конкретным светодиодам
 */
typedef enum
{
    BSP_LEDC_LED_VD1 = 0,     ///< VD1
    BSP_LEDC_LED_VD2,      	  ///< VD2
    BSP_LEDC_LED_COUNT        ///< Количество светодиодов (служебное значение)
} BSP_LedcLeds;

/**
 * @brief Режимы работы светодиодов
 */
typedef enum
{
    BSP_LEDC_MODE_OFF = 0,    ///< Светодиод выключен
    BSP_LEDC_MODE_ON,         ///< Светодиод включен
    BSP_LEDC_MODE_1,          ///< Пользовательский режим 1
    BSP_LEDC_MODE_2,          ///< Пользовательский режим 2
    BSP_LEDC_MODE_COUNT       ///< Количество режимов (служебное значение)
} BSP_LedcModes;

/**
 * @brief Инициализация контроллера светодиодов
 *
 * Функция выполняет полную инициализацию подсистемы управления светодиодами,
 * включая создание очереди команд и запуск управляющего потока.
 *
 * @return bool
 *   - true: успешная инициализация
 *   - false: ошибка при инициализации
 *
 * @note Функция должна вызываться до начала использования других функций управления светодиодами
 *
 * @warning При сбое инициализации (возврат false) использование системы светодиодов недопустимо
 */
bool Bsp_LedC_Init_If(void);

/**
 * @brief Установка режима работы светодиода
 * @param led Идентификатор светодиода (из BSP_LedcLeds)
 * @param mode Режим работы (из BSP_LedcModes)
 * @return true - команда принята, false - очередь переполнена (требуется повторный вызов)
 * @note Функция не блокирует выполнение. Для гарантированной доставки команды
 *       рекомендуется реализовать повторные попытки на уровне вызывающего кода.
 */
bool Bsp_LedC_SetLedMode_If(BSP_LedcLeds led, BSP_LedcModes mode);

#endif /* INC_LEDSCONTROLLER_IF_H_ */
