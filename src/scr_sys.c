/*
*  SCRx FreeRTOS Demo
*  @copyright (C) Syntacore 2019. All rights reserved.
*  @brief FreeRTOS SCR-specific handlers and more
*/

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
//#include "drivers/rtc.h"
#include "riscv_csr_encoding.h"
//#include <scr1_csr_encoding.h>
#include "scr1_timer.h"
#include <mik32_memory_map.h>

//static const unsigned long rtc_ticks_per_timer_shot = PLF_HZ / configTICK_RATE_HZ;
//static sys_tick_t next_rtc_timer_shot = 0;

void vPortSetupTimerInterrupt( void )
{
//#if 0
//    rtc_setcmp_offset(rtc_us2ticks(1000000 / configTICK_RATE_HZ));
//#else
//    sys_tick_t t = rtc_now() + rtc_ticks_per_timer_shot;
//    rtc_setcmp(t);
//    next_rtc_timer_shot = t + rtc_ticks_per_timer_shot;
//#endif
//    /* enable timer interrupts */
//    rtc_interrupt_enable();
	SCR1_TIMER->TIMER_DIV = 0;
	*(unsigned long long *) &SCR1_TIMER->MTIME = 0;
	*(unsigned long long *) &SCR1_TIMER->MTIMECMP = configCPU_CLOCK_HZ/configTICK_RATE_HZ;


	SCR1_TIMER->TIMER_CTRL |= SCR1_TIMER_CTRL_ENABLE_M;

	set_csr(mie, (1<<7));
}

void ext_trap_handler(void);

void trap_handler_entry(void)
{
//#if 0
//    rtc_setcmp_offset(rtc_us2ticks(1000000 / configTICK_RATE_HZ));
//#else
//    sys_tick_t t = next_rtc_timer_shot;
//    rtc_setcmp(t);
//    next_rtc_timer_shot = t + rtc_ticks_per_timer_shot;
//#endif

	unsigned long temp_mcause = read_csr(mcause);
	if ( (temp_mcause & 0xF) == 7 && (temp_mcause & (1<<31)) )
	{
		*(unsigned long long *) &SCR1_TIMER->MTIMECMP += configCPU_CLOCK_HZ/configTICK_RATE_HZ;

		if (xTaskIncrementTick() != pdFALSE)
			vTaskSwitchContext();
	}
	else
	{
		ext_trap_handler();
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
    //printf("\nApplication Malloc Failed!\n");
    vTaskEndScheduler();
}
/*-----------------------------------------------------------*/

void vAssertCalled( void )
{
    register unsigned long ra asm ("ra");

	taskDISABLE_INTERRUPTS();
    //printf("\nvAssertCalled (ret addr= 0x%lx)!\n", ra);

    vTaskEndScheduler();
}
