/*
 * ports_driver.h
 *
 *  Created on: 9 июл. 2025 г.
 *      Author: user
 */

#ifndef DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_
#define DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_


#include <stdbool.h>
#include <stddef.h>



bool Bsp_Ports_Init_If(void);


void Bsp_VD2_On_If(void);
void Bsp_VD2_Off_If(void);

void Bsp_PwrOnPort_On_If(void);
void Bsp_PwrOnPort_Off_If(void);

bool Bsp_PrstPort_Read_If(void);
bool Bsp_VD2_Read_If(void);
bool Bsp_PwrOnPort_Read_If(void);


#endif /* DRIVERS_PORTS_DRIVER_PORTS_DRIVER_H_ */
