/**
 * @file app.h
 * @brief Заголовочный файл модуля приложения
 * @author Evgenii Shkliaev
 * @date 18-06-2025
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "wdt_driver.h"
#include "BspLedsController_If.h"
#include "BspUartTransport_If.h"
#include "i2c_driver_master.h"


#include "FreeRTOS.h"
#include "task.h"

#include <string.h>


/**
 * @brief Инициализация приложения
 * @return true если инициализация прошла успешно, false в противном случае
 */
bool App_Init(void);


#endif /* INC_APP_H_ */
