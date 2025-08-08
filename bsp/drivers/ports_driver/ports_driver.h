/**
 * @file ports_driver.h
 * @brief Интерфейс драйвера для управления портами ввода/вывода.
 */

#ifndef DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_
#define DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Инициализация портов ввода/вывода.
 * @return true, если инициализация прошла успешно, иначе false.
 */
bool Bsp_Ports_Init_If(void);

/**
 * @brief Включение VD2.
 */
void Bsp_VD2_On_If(void);

/**
 * @brief Выключение VD2.
 */
void Bsp_VD2_Off_If(void);

/**
 * @brief Включение порта питания.
 */
void Bsp_PwrOnPort_On_If(void);

/**
 * @brief Выключение порта питания.
 */
void Bsp_PwrOnPort_Off_If(void);

/**
 * @brief Чтение состояния порта Prst.
 * @return Состояние порта (true/false).
 */
bool Bsp_PrstPort_Read_If(void);

/**
 * @brief Чтение состояния VD2.
 * @return Состояние VD2 (true/false).
 */
bool Bsp_VD2_Read_If(void);

/**
 * @brief Чтение состояния порта питания.
 * @return Состояние порта питания (true/false).
 */
bool Bsp_PwrOnPort_Read_If(void);

/**
 * @brief Чтение состояния кнопки пользователя.
 * @return Состояние кнопки (true/false).
 */
bool Bsp_UserBtn_Read_If(void);

#endif /* DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_ */
