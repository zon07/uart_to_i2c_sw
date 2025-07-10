/**
 * @file app.h
 * @brief Заголовочный файл модуля приложения
 * @author Evgenii Shkliaev
 * @date 18-06-2025
 */

#ifndef INC_APP_H_
#define INC_APP_H_


#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdbool.h>


/**
 * @brief Инициализация приложения
 * @return true если инициализация прошла успешно, false в противном случае
 */
bool App_Init(void);


#endif /* INC_APP_H_ */
