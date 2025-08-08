/*
 * wdt_driver.h
 *
 *  Created on: 11 июн. 2025 г.
 *      Author: user
 */

#ifndef INC_WDT_DRIVER_H_
#define INC_WDT_DRIVER_H_

#include "mik32_memory_map.h"
#include "power_manager.h"
#include "wdt.h"
#include <stdbool.h>




bool WDT_Init(void);
void WDT_Reset(void);
bool WDT_CheckResetFlag(void);


#endif /* INC_WDT_DRIVER_H_ */
