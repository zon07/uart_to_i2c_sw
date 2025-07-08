#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

//#include "arch.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configCLINT_BASE_ADDRESS				0 /*CLINT_CTRL_ADDR*/
#define configUSE_TASK_FPU_SUPPORT              0

#define configUSE_PREEMPTION					1
#define configUSE_TIME_SCILING                  1

#define configUSE_IDLE_HOOK						0
#define configUSE_TICK_HOOK						0
#define configCPU_CLOCK_HZ						( 32000000 ) /* not used for SCRx cores */
#define configTICK_RATE_HZ	             	 	( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES					( 7 )
#define configMINIMAL_STACK_SIZE				( ( unsigned short ) 128 )
#define configISR_STACK_SIZE_WORDS				128*4
#define configTOTAL_HEAP_SIZE					( ( size_t ) 10*1024 )

#define configMAX_TASK_NAME_LEN					( 16 )
#define configUSE_TRACE_FACILITY				0
#define configUSE_16_BIT_TICKS					0
#define configIDLE_SHOULD_YIELD					0
#define configUSE_MUTEXES						1
#define configQUEUE_REGISTRY_SIZE				8

#define configCHECK_FOR_STACK_OVERFLOW	        0

#define configUSE_RECURSIVE_MUTEXES				1
#define configUSE_MALLOC_FAILED_HOOK	        1
#define configUSE_APPLICATION_TASK_TAG	        0
#define configUSE_COUNTING_SEMAPHORES	        1
#define configGENERATE_RUN_TIME_STATS	        0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#define configSUPPORT_STATIC_ALLOCATION         0    // 1 - статическое выделение памяти
#define configSUPPORT_DYNAMIC_ALLOCATION        1    // 1 - динамическое выделение (куча)
#define configAPPLICATION_ALLOCATED_HEAP        0    // 1 - куча выделяется вручную

#define configUSE_MPU_WRAPPERS                  0    // 1 - включить обёртки MPU
#define configENABLE_MPU                        0    // 1 - аппаратная защита памяти

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 					0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS						1
#define configTIMER_TASK_PRIORITY				( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH				4
#define configTIMER_TASK_STACK_DEPTH	        ( configMINIMAL_STACK_SIZE )

/* Task priorities.  Allow these to be overridden. */
#ifndef uartPRIMARY_PRIORITY
	#define uartPRIMARY_PRIORITY				( configMAX_PRIORITIES - 3 )
#endif

/* Set the following definitions to 1 to include the API function, or zero
 to exclude the API function. */
#define INCLUDE_vTaskPrioritySet				1
#define INCLUDE_uxTaskPriorityGet				1
#define INCLUDE_vTaskDelete						1
#define INCLUDE_vTaskCleanUpResources	        1
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil					1
#define INCLUDE_vTaskDelay						1
#define INCLUDE_eTaskGetState					1
#define INCLUDE_xTimerPendFunctionCall	        1
#define INCLUDE_xTaskGetCurrentTaskHandle		1

#define INCLUDE_uxTaskGetStackHighWaterMark 	1

/* Normal assert() semantics without relying on the provision of an assert.h
 header file. */
void vAssertCalled(void);
#define configASSERT( x ) do { if( ( x ) == 0 ) vAssertCalled(); } while (0)

/* Overwrite some of the stack sizes allocated to various test and demo tasks.
 Like all task stack sizes, the value is the number of words, not bytes. */
#define bktBLOCK_TIME_TASK_STACK_SIZE                           100
#define notifyNOTIFIED_TASK_STACK_SIZE                          120
#define priSUSPENDED_RX_TASK_STACK_SIZE                         90
#define tmrTIMER_TEST_TASK_STACK_SIZE                           100
#define ebRENDESVOUS_TEST_TASK_STACK_SIZE                       100
#define ebEVENT_GROUP_SET_BITS_TEST_TASK_STACK_SIZE             115
#define genqMUTEX_TEST_TASK_STACK_SIZE                          90
#define genqGENERIC_QUEUE_TEST_TASK_STACK_SIZE                  100
#define recmuRECURSIVE_MUTEX_TEST_TASK_STACK_SIZE               90

#endif /* FREERTOS_CONFIG_H */
