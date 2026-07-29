#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "cy_utils.h"

extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configCPU_CLOCK_HZ SystemCoreClock
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES 7
#define configMINIMAL_STACK_SIZE 256
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configQUEUE_REGISTRY_SIZE 0
#define configUSE_QUEUE_SETS 0
#define configUSE_TIME_SLICING 1
#define configENABLE_BACKWARD_COMPATIBILITY 0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0

#define configENABLE_FPU 1
#define configENABLE_MVE 0
#define configENABLE_MPU 0
#define configENABLE_TRUSTZONE 0
#define configRUN_FREERTOS_SECURE_ONLY 0

#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE ((size_t)(64U * 1024U))
#define configAPPLICATION_ALLOCATED_HEAP 0
#define configHEAP_ALLOCATION_SCHEME HEAP_ALLOCATION_TYPE4

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES 1
#define configUSE_TIMERS 0
#define configUSE_NEWLIB_REENTRANT 0
#define configUSE_TICKLESS_IDLE 0

#define configMAX_SYSCALL_INTERRUPT_PRIORITY 0x20
#define configMAX_API_CALL_INTERRUPT_PRIORITY \
    configMAX_SYSCALL_INTERRUPT_PRIORITY

#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 0
#define INCLUDE_xResumeFromISR 0
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 0
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 0
#define INCLUDE_xTaskGetIdleTaskHandle 0
#define INCLUDE_eTaskGetState 0
#define INCLUDE_xEventGroupSetBitFromISR 0
#define INCLUDE_xTimerPendFunctionCall 0
#define INCLUDE_xTaskAbortDelay 0
#define INCLUDE_xTaskGetHandle 0
#define INCLUDE_xTaskResumeFromISR 0

#define configASSERT(value) \
    do { if ((value) == 0) { taskDISABLE_INTERRUPTS(); CY_HALT(); } } while (0)

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define HEAP_ALLOCATION_TYPE1 1
#define HEAP_ALLOCATION_TYPE2 2
#define HEAP_ALLOCATION_TYPE3 3
#define HEAP_ALLOCATION_TYPE4 4
#define HEAP_ALLOCATION_TYPE5 5
#define NO_HEAP_ALLOCATION 0

#endif
