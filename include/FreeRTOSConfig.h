#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * TMS570LS1224 Main ECU -- clean rewrite.
 *-----------------------------------------------------------*/

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_FPU                           1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configCPU_CLOCK_HZ                      ( ( unsigned portLONG ) 80000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 8 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned portSHORT ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) 8192 )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configIDLE_SHOULD_YIELD                 1
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_MALLOC_FAILED_HOOK            0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
/* No tickless idle: this port doesn't implement
 * portSUPPRESS_TICKS_AND_SLEEP, and there's no need for it here. */
#define configUSE_TICKLESS_IDLE                 0

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 0 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( 128 )

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskGetIdleTaskHandle           1
#define INCLUDE_xTimerPendFunctionCall           1

/* Halt with interrupts disabled so a debugger sees exactly where an
 * assertion failed, rather than continuing into undefined behaviour. */
#define configASSERT( x ) if( ( x ) == pdFALSE ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#endif /* FREERTOS_CONFIG_H */
