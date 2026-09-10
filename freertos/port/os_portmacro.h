/*
 * Minimal FreeRTOS port layer for TMS570LS1224 (Cortex-R4F, big-endian,
 * VFPv3-D16), GCC toolchain.
 *
 * Written from scratch for this project. No MPU wrappers: every task runs
 * privileged (System mode) with a single flat memory map. That removes an
 * entire class of bugs the previous port had (unprivileged tasks silently
 * unable to mask interrupts, MPU region reprogramming from uninitialized
 * TCB data, etc.) -- this application has no need for per-task memory
 * isolation.
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  uint32_t
#define portBASE_TYPE   long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if ( configUSE_16_BIT_TICKS == 1 )
    typedef uint16_t TickType_t;
    #define portMAX_DELAY ( TickType_t ) 0xFFFF
#else
    typedef uint32_t TickType_t;
    #define portMAX_DELAY ( TickType_t ) 0xFFFFFFFFUL
    #define portTICK_TYPE_IS_ATOMIC 1
#endif

/* Architecture specifics. */
#define portSTACK_GROWTH        ( -1 )
#define portTICK_PERIOD_MS      ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT      8

/* No MPU wrappers: every task is privileged, flat memory map. This bit is
 * still OR'd unconditionally into the idle/timer task priority by
 * os_tasks.c regardless of portUSING_MPU_WRAPPERS, so it must be defined --
 * as 0 (a no-op) rather than a real flag, since there is no privilege
 * concept to strip back out afterward. */
#define portUSING_MPU_WRAPPERS  0
#define portPRIVILEGE_BIT       ( 0x00000000UL )

/* Critical section handling: tasks are always privileged (System mode), so
 * CPSID/CPSIE work directly -- no SVC bounce needed. This closes the exact
 * hole the previous port had (SVC entry doesn't itself mask IRQ, so a tick
 * could preempt mid-critical-section-entry). */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );
extern void vPortDisableInterrupts( void );
extern void vPortEnableInterrupts( void );

#define portENTER_CRITICAL()      vPortEnterCritical()
#define portEXIT_CRITICAL()       vPortExitCritical()
#define portDISABLE_INTERRUPTS()  vPortDisableInterrupts()
#define portENABLE_INTERRUPTS()   vPortEnableInterrupts()

/* Scheduler utilities. Yield is a plain SVC (synchronous, always taken --
 * no VIM channel / software-interrupt routing to get wrong). */
extern void vPortYield( void );
#define portYIELD()  vPortYield()

/* Used to force a reschedule from within an API call while interrupts are
 * already disabled (task self-delete/self-suspend). Since we're always
 * privileged, this is just... yield; the SVC itself doesn't care about the
 * caller's I-bit state. */
#define portYIELD_WITHIN_API()  vPortYield()

/* Not used by this application (no peripheral interrupt calls a FromISR
 * API -- CAN RX is a polling task), but must compile: a plain yield is a
 * safe, always-correct implementation since we're always privileged. */
#define portYIELD_FROM_ISR( x )     do { if( ( x ) != 0 ) { vPortYield(); } } while( 0 )
#define portEND_SWITCHING_ISR( x )  portYIELD_FROM_ISR( x )

#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

    unsigned long ulPortCountLeadingZeros( unsigned long ulBitmap );

    #if( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.
    #endif

    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )
    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31 - ulPortCountLeadingZeros( ( uxReadyPriorities ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* Task function macros. */
#define portTASK_FUNCTION( vFunction, pvParameters )       void vFunction( void *pvParameters )
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )

#define portNOP()  __asm volatile( "NOP" )

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
