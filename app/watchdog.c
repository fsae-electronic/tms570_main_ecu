/*
 * Digital watchdog (RTI DWD) for the Main ECU.
 *
 * A high-priority task wakes every WATCHDOG_PERIOD_MS and refreshes the
 * DWD only if every task in WatchdogTask_t has checked in during the last
 * WATCHDOG_CHECKIN_MAX_MS. The watchdog task itself always gets the CPU
 * (highest priority), so a missing check-in means that task is hung or
 * starved, and a missing refresh means the whole CPU is (ISR storm,
 * interrupts disabled, data abort loop). Either way the DWD expires and
 * resets the MCU; after the reset the drives are sent to pre-operational
 * by init_drivers() and drive stays off until a new cmd 7.
 *
 * Once DWDCTRL is written the DWD can't be stopped until the next reset.
 * Set WATCHDOG_ENABLE to 0 for debugging sessions with breakpoints if the
 * halted CPU gets reset by it.
 */

#include "watchdog.h"

#include "os_task.h"
#include "rti.h"

#include <stdbool.h>

#define WATCHDOG_ENABLE           1

#define WATCHDOG_PERIOD_MS        20U
/* The drivers loop is the slowest: 10 ms period plus ~15 ms of the CiA 402
 * enable sequence on drive-on. */
#define WATCHDOG_CHECKIN_MAX_MS   60U

/* DWD expiration = (preload + 1) * 2^13 / RTICLK. RTICLK is VCLK (80 MHz)
 * or VCLK / 2 depending on RCLKSRC: 976 gives ~100 ms at 80 MHz and
 * ~200 ms at 40 MHz. Max preload is 4095. */
#define WATCHDOG_DWD_PRELOAD      976U

#define WATCHDOG_STACK            128U

static StaticTask_t xWatchdogTaskBuffer;
static StackType_t xWatchdogStack[ WATCHDOG_STACK ];

static volatile TickType_t xLastCheckin[ WATCHDOG_TASK_COUNT ];

void watchdog_checkin( WatchdogTask_t eTask )
{
    if( eTask < WATCHDOG_TASK_COUNT )
    {
        xLastCheckin[ eTask ] = xTaskGetTickCount();
    }
}

static bool bAllTasksAlive( TickType_t xNow )
{
    uint32_t n;

    for( n = 0U; n < ( uint32_t ) WATCHDOG_TASK_COUNT; n++ )
    {
        if( ( xNow - xLastCheckin[ n ] ) > pdMS_TO_TICKS( WATCHDOG_CHECKIN_MAX_MS ) )
        {
            return false;
        }
    }

    return true;
}

static portTASK_FUNCTION( vWatchdogTask, pvParameters )
{
    uint32_t n;
    ( void ) pvParameters;

    /* Every task gets a full check-in window from here. */
    for( n = 0U; n < ( uint32_t ) WATCHDOG_TASK_COUNT; n++ )
    {
        xLastCheckin[ n ] = xTaskGetTickCount();
    }

#if WATCHDOG_ENABLE
    dwwdInit( Generate_Reset, ( uint16 ) WATCHDOG_DWD_PRELOAD, Size_100_Percent );
    dwdCounterEnable();
#endif

    for( ;; )
    {
        if( bAllTasksAlive( xTaskGetTickCount() ) )
        {
#if WATCHDOG_ENABLE
            dwdReset();
#endif
        }

        vTaskDelay( pdMS_TO_TICKS( WATCHDOG_PERIOD_MS ) );
    }
}

void watchdog_init( UBaseType_t uxPriority )
{
    xTaskCreateStatic( vWatchdogTask, "Watchdog", WATCHDOG_STACK, NULL,
                       uxPriority, xWatchdogStack, &xWatchdogTaskBuffer );
}
