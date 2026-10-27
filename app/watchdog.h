#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "FreeRTOS.h"

/* Tasks that must check in for the watchdog to be fed. If any of them
 * stops running (hung in a busy-wait, starved, stuck with interrupts off)
 * the DWD isn't refreshed and the MCU resets. */
typedef enum
{
    WATCHDOG_TASK_CAN_RX = 0,
    WATCHDOG_TASK_DRIVERS,
    WATCHDOG_TASK_COUNT
} WatchdogTask_t;

/* Creates the watchdog task. The DWD itself is only started from that
 * task, once the scheduler runs, so the boot isn't timed. */
void watchdog_init( UBaseType_t uxPriority );

/* Call once per loop iteration of each WatchdogTask_t task. */
void watchdog_checkin( WatchdogTask_t eTask );

#endif /* WATCHDOG_H_ */
