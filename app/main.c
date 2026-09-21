/*
 * Main ECU application -- clean rewrite.
 *
 * Pipeline: CAN RX task polls every mailbox this ECU cares about (see
 * telemetry.h / the "Mensajes CAN" spec / can-logger/cansniff/messages.py
 * for the catalogue) and updates a mutex-protected snapshot; the telemetry
 * task wakes periodically, takes a consistent copy of that snapshot, and
 * sends it out over SCI/UART as a fixed-size frame (see vPackFrame for the
 * exact layout -- keep tools/decode_telemetry.py in sync with it). There
 * is a placeholder for SAE compliance rules between those two steps (not
 * implemented yet, by design -- see the comment at vTelemetryTask).
 *
 * CAN is polled, not interrupt-driven: canGetData() itself checks the
 * hardware NEWDAT flag and returns 0 if there's nothing new, so a short
 * vTaskDelay loop is a simple, robust way to pick up new frames without
 * needing any VIM channel routing for CAN at all.
 *
 * All multi-byte CAN signal fields are little-endian on the wire (B0 =
 * LSB) -- see telemetry.h for why that's now confirmed rather than
 * assumed.
 */

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"

#include "can.h"
#include "sci.h"
#include "gio.h"
#include "drivers.h"

#include "telemetry.h"

#include <string.h>

/* ---- Configuration ------------------------------------------------- */

#define PRIO_CAN_RX       3U
#define PRIO_DRIVERS      2U
#define PRIO_TELEMETRY    1U

#define STACK_CAN_RX       256U
#define STACK_DRIVERS      256U
#define STACK_TELEMETRY    256U

#define CAN_RX_PERIOD_MS      10U
#define DRIVERS_PERIOD_MS     10U
#define TELEMETRY_PERIOD_MS  100U

#define CALIBRATION_CMD_DRIVE_ENABLE_ON  7U
#define CALIBRATION_CMD_DRIVE_ENABLE_OFF 8U
#define CALIBRATION_CMD_TELEMETRY 0xAU

/* ---- Little-endian CAN payload decoding helpers ---------------------- */

static uint16_t leU16( const uint8 *p )
{
    return ( uint16_t ) ( ( uint16_t ) p[ 0 ] | ( ( uint16_t ) p[ 1 ] << 8 ) );
}

static int16_t leI16( const uint8 *p )
{
    return ( int16_t ) leU16( p );
}

static uint32_t leU32( const uint8 *p )
{
    return ( uint32_t ) p[ 0 ]
         | ( ( uint32_t ) p[ 1 ] << 8 )
         | ( ( uint32_t ) p[ 2 ] << 16 )
         | ( ( uint32_t ) p[ 3 ] << 24 );
}

static int32_t leI32( const uint8 *p )
{
    return ( int32_t ) leU32( p );
}

/* ---- Shared state ---------------------------------------------------*/

TelemetrySnapshot_t xTelemetryData;
SemaphoreHandle_t xTelemetryMutex;
static bool xTelemetryEnabled = true;

static StaticSemaphore_t xTelemetryMutexBuffer;

static StaticTask_t xCanRxTaskBuffer;
static StackType_t xCanRxStack[ STACK_CAN_RX ];

static StaticTask_t xDriversTaskBuffer;
static StackType_t xDriversStack[ STACK_DRIVERS ];

static StaticTask_t xTelemetryTaskBuffer;
static StackType_t xTelemetryStack[ STACK_TELEMETRY ];

/* ---- Tasks ------------------------------------------------------------*/

/* One CANopen driver node's PDOs land on 5 separate mailboxes (RXPDO1,
 * TXPDO1-3, heartbeat); this decodes whichever of those has new data into
 * the matching DriverNode_t slot. `node` is 0 (left) or 1 (right), and the
 * mailbox numbers follow canInit()'s layout (boxes 14-23). */
static void vPollDriverNode( uint32 node, uint32 boxTxPdo1, uint32 boxTxPdo2,
                              uint32 boxTxPdo3, uint32 boxRxPdo1, uint32 boxHeartbeat )
{
    uint8 raw[ 8 ];

    if( canGetData( canREG1, boxTxPdo1, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].status_word = leU16( &raw[ 0 ] );
        xTelemetryData.driver[ node ].warnings    = leU16( &raw[ 2 ] );
        xTelemetryData.driver[ node ].error       = leU16( &raw[ 4 ] );
        xTelemetryData.driver[ node ].txpdo1_valid = true;
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxTxPdo2, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].motor_velocity     = leI32( &raw[ 0 ] );
        xTelemetryData.driver[ node ].motor_rated_current = leU32( &raw[ 4 ] );
        xTelemetryData.driver[ node ].txpdo2_valid = true;
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxTxPdo3, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].driver_temp       = raw[ 0 ];
        xTelemetryData.driver[ node ].driver_dc_voltage = leU16( &raw[ 1 ] );
        xTelemetryData.driver[ node ].motor_temp         = raw[ 3 ];
        xTelemetryData.driver[ node ].txpdo3_valid = true;
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxRxPdo1, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].control_word  = leU16( &raw[ 0 ] );
        xTelemetryData.driver[ node ].target_torque = leI16( &raw[ 2 ] );
        xTelemetryData.driver[ node ].rxpdo1_valid = true;
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxHeartbeat, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].heartbeat_state = raw[ 0 ];
        xTelemetryData.driver[ node ].heartbeat_valid = true;
        xSemaphoreGive( xTelemetryMutex );
    }
}

static portTASK_FUNCTION( vCanRxTask, pvParameters )
{
    uint8 raw[ 8 ];
    ( void ) pvParameters;

    for( ;; )
    {
        if( canGetData( canREG1, canMESSAGE_BOX1, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.sensors.tps1 = (uint16_t)(raw[ 1 ] | (raw[ 0 ] << 8 ) );
            xTelemetryData.sensors.tps2 = (uint16_t)(raw[ 3 ] | (raw[ 2 ] << 8 ) );
            xTelemetryData.sensors_valid = true;
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX2, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.front_axle.wheel_speed_fl = leU16( &raw[ 0 ] );
            xTelemetryData.front_axle.wheel_speed_fr = leU16( &raw[ 2 ] );
            xTelemetryData.front_axle.steering_angle = leU16( &raw[ 4 ] );
            xTelemetryData.front_axle.front_brake    = leU16( &raw[ 6 ] );
            xTelemetryData.front_axle_valid = true;
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX12, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.current.ac_current_1 = leU16( &raw[ 0 ] );
            xTelemetryData.current.ac_current_2 = leU16( &raw[ 2 ] );
            xTelemetryData.current.dc_current_1 = leU16( &raw[ 4 ] );
            xTelemetryData.current.dc_current_2 = leU16( &raw[ 6 ] );
            xTelemetryData.current_valid = true;
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX13, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.rear.rear_brake = leU16( &raw[ 0 ] );
            xTelemetryData.rear_valid = true;
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX24, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.calibration_type = raw[ 0 ];
            xTelemetryData.calibration_value = raw[ 1 ];
            xTelemetryData.calibration_valid = true;
            if( raw[ 0 ] == CALIBRATION_CMD_TELEMETRY )
            {
                xTelemetryEnabled = ( raw[ 1 ] != 0U );
            }
            xSemaphoreGive( xTelemetryMutex );
        }

        vPollDriverNode( 0U, canMESSAGE_BOX14, canMESSAGE_BOX16, canMESSAGE_BOX18,
                          canMESSAGE_BOX20, canMESSAGE_BOX22 );
        vPollDriverNode( 1U, canMESSAGE_BOX15, canMESSAGE_BOX17, canMESSAGE_BOX19,
                          canMESSAGE_BOX21, canMESSAGE_BOX23 );

        vTaskDelay( pdMS_TO_TICKS( CAN_RX_PERIOD_MS ) );
    }
}

/* ---- Wire frame -------------------------------------------------------
 *
 * Every multi-byte field is little-endian, matching the CAN payloads
 * themselves (see telemetry.h). Layout (keep tools/decode_telemetry.py's
 * struct format string in lockstep with this):
 *
 *   offset  size  field
 *   0       2     sync (0xAA 0x55)
 *   2       1     version (2)
 *   3       2     valid_mask (bit set per message actually received at
 *                 least once -- see TELEMETRY_VALID_* below)
 *   5       2     tps1
 *   7       2     tps2
 *   9       2     wheel_speed_fl
 *   11      2     wheel_speed_fr
 *   13      2     steering_angle
 *   15      2     front_brake
 *   17      2     ac_current_1
 *   19      2     ac_current_2
 *   21      2     dc_current_1
 *   23      2     dc_current_2
 *   25      2     rear_brake
 *   27      1     calibration_type
 *   28      ...   per driver node (left then right), 23 bytes each:
 *                   control_word(2) target_torque(2) status_word(2)
 *                   warnings(2) error(2) motor_velocity(4)
 *                   motor_rated_current(4) driver_temp(1)
 *                   driver_dc_voltage(2) motor_temp(1) heartbeat_state(1)
 *   74      --    end
 */
#define TELEMETRY_FRAME_SIZE 74U

#define TELEMETRY_VALID_TPS        ( 1U << 0 )
#define TELEMETRY_VALID_FRONT      ( 1U << 1 )
#define TELEMETRY_VALID_CURRENT    ( 1U << 2 )
#define TELEMETRY_VALID_REAR       ( 1U << 3 )
#define TELEMETRY_VALID_CAL        ( 1U << 4 )
#define TELEMETRY_VALID_RXPDO1( n )    ( 1U << ( 5U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO1( n )    ( 1U << ( 6U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO2( n )    ( 1U << ( 7U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO3( n )    ( 1U << ( 8U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_HEARTBEAT( n ) ( 1U << ( 9U + ( 5U * ( n ) ) ) )

static void putU8( uint8_t *pxFrame, uint32_t *pIdx, uint8_t v )
{
    pxFrame[ *pIdx ] = v;
    *pIdx += 1U;
}

static void putU16( uint8_t *pxFrame, uint32_t *pIdx, uint16_t v )
{
    pxFrame[ *pIdx ]     = ( uint8_t ) ( v & 0xFFU );
    pxFrame[ *pIdx + 1 ] = ( uint8_t ) ( v >> 8 );
    *pIdx += 2U;
}

static void putU32( uint8_t *pxFrame, uint32_t *pIdx, uint32_t v )
{
    pxFrame[ *pIdx ]     = ( uint8_t ) ( v & 0xFFU );
    pxFrame[ *pIdx + 1 ] = ( uint8_t ) ( ( v >> 8 ) & 0xFFU );
    pxFrame[ *pIdx + 2 ] = ( uint8_t ) ( ( v >> 16 ) & 0xFFU );
    pxFrame[ *pIdx + 3 ] = ( uint8_t ) ( v >> 24 );
    *pIdx += 4U;
}

static void vPackFrame( const TelemetrySnapshot_t *pxSnapshot, uint8_t *pxFrame )
{
    uint32_t ulMask = 0U;
    uint32_t n;
    uint32_t idx = 0U;

    if( pxSnapshot->sensors_valid )     { ulMask |= TELEMETRY_VALID_TPS; }
    if( pxSnapshot->front_axle_valid )  { ulMask |= TELEMETRY_VALID_FRONT; }
    if( pxSnapshot->current_valid )     { ulMask |= TELEMETRY_VALID_CURRENT; }
    if( pxSnapshot->rear_valid )        { ulMask |= TELEMETRY_VALID_REAR; }
    if( pxSnapshot->calibration_valid ) { ulMask |= TELEMETRY_VALID_CAL; }
    for( n = 0U; n < 2U; n++ )
    {
        if( pxSnapshot->driver[ n ].rxpdo1_valid )    { ulMask |= TELEMETRY_VALID_RXPDO1( n ); }
        if( pxSnapshot->driver[ n ].txpdo1_valid )    { ulMask |= TELEMETRY_VALID_TXPDO1( n ); }
        if( pxSnapshot->driver[ n ].txpdo2_valid )    { ulMask |= TELEMETRY_VALID_TXPDO2( n ); }
        if( pxSnapshot->driver[ n ].txpdo3_valid )    { ulMask |= TELEMETRY_VALID_TXPDO3( n ); }
        if( pxSnapshot->driver[ n ].heartbeat_valid ) { ulMask |= TELEMETRY_VALID_HEARTBEAT( n ); }
    }

    putU8( pxFrame, &idx, 0xAAU );
    putU8( pxFrame, &idx, 0x55U );
    putU8( pxFrame, &idx, 2U ); /* version */
    putU16( pxFrame, &idx, ( uint16_t ) ulMask );

    putU16( pxFrame, &idx, pxSnapshot->sensors.tps1 );
    putU16( pxFrame, &idx, pxSnapshot->sensors.tps2 );

    putU16( pxFrame, &idx, pxSnapshot->front_axle.wheel_speed_fl );
    putU16( pxFrame, &idx, pxSnapshot->front_axle.wheel_speed_fr );
    putU16( pxFrame, &idx, pxSnapshot->front_axle.steering_angle );
    putU16( pxFrame, &idx, pxSnapshot->front_axle.front_brake );

    putU16( pxFrame, &idx, pxSnapshot->current.ac_current_1 );
    putU16( pxFrame, &idx, pxSnapshot->current.ac_current_2 );
    putU16( pxFrame, &idx, pxSnapshot->current.dc_current_1 );
    putU16( pxFrame, &idx, pxSnapshot->current.dc_current_2 );

    putU16( pxFrame, &idx, pxSnapshot->rear.rear_brake );

    putU8( pxFrame, &idx, pxSnapshot->calibration_type );

    for( n = 0U; n < 2U; n++ )
    {
        const DriverNode_t *pxNode = &pxSnapshot->driver[ n ];

        putU16( pxFrame, &idx, pxNode->control_word );
        putU16( pxFrame, &idx, ( uint16_t ) pxNode->target_torque );
        putU16( pxFrame, &idx, pxNode->status_word );
        putU16( pxFrame, &idx, pxNode->warnings );
        putU16( pxFrame, &idx, pxNode->error );
        putU32( pxFrame, &idx, ( uint32_t ) pxNode->motor_velocity );
        putU32( pxFrame, &idx, pxNode->motor_rated_current );
        putU8( pxFrame, &idx, pxNode->driver_temp );
        putU16( pxFrame, &idx, pxNode->driver_dc_voltage );
        putU8( pxFrame, &idx, pxNode->motor_temp );
        putU8( pxFrame, &idx, pxNode->heartbeat_state );
    }
}

static portTASK_FUNCTION( vTelemetryTask, pvParameters )
{
    TelemetrySnapshot_t xSnapshot;
    uint8_t xFrame[ TELEMETRY_FRAME_SIZE ];
    bool bTelemetryEnabled;
    ( void ) pvParameters;

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( TELEMETRY_PERIOD_MS ) );

        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xSnapshot = xTelemetryData;
        xSemaphoreGive( xTelemetryMutex );

        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        bTelemetryEnabled = xTelemetryEnabled;
        xSemaphoreGive( xTelemetryMutex );

        if( !bTelemetryEnabled )
        {
            continue;
        }

        /* TODO: SAE compliance rules go here, between reading the
         * snapshot and sending it -- deliberately not implemented yet.
         * For now this is a straight pass-through. */

        vPackFrame( &xSnapshot, xFrame );
        sciSend( sciREG, TELEMETRY_FRAME_SIZE, xFrame );
    }
}

static portTASK_FUNCTION( vDriversTask, pvParameters )
{
    bool bDriveEnabled = false;
    int16_t s16Torque = 0;
    ( void ) pvParameters;

    for( ;; )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );

        /* CAN ID 0x600: Byte 0 is the command id. */
        if( xTelemetryData.calibration_valid )
        {
            if( xTelemetryData.calibration_type == CALIBRATION_CMD_DRIVE_ENABLE_ON )
            {
                bDriveEnabled = true;
            }
            else if( xTelemetryData.calibration_type == CALIBRATION_CMD_DRIVE_ENABLE_OFF )
            {
                bDriveEnabled = false;
            }
        }

        /* Torque is the average between APPS1 (tps1) and APPS2 (tps2) */
        if( xTelemetryData.sensors_valid )
        {
            uint32_t ulAvgTps = ( ( uint32_t ) xTelemetryData.sensors.tps1 + ( uint32_t ) xTelemetryData.sensors.tps2 ) / 2U;
            s16Torque = ( int16_t ) ulAvgTps;
        }
        else
        {
            s16Torque = 0;
        }

        xSemaphoreGive( xTelemetryMutex );

        run_drivers( bDriveEnabled, s16Torque );

        vTaskDelay( pdMS_TO_TICKS( DRIVERS_PERIOD_MS ) );
    }
}

/* ---- Static allocation callbacks required by configSUPPORT_STATIC_ALLOCATION */

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                     StackType_t **ppxIdleTaskStackBuffer,
                                     uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                      StackType_t **ppxTimerTaskStackBuffer,
                                      uint32_t *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* ---- Entry point ------------------------------------------------------*/

int main( void )
{
    const char *pcBootMsg = "TMS570 MAIN ECU (rewrite) INITIALIZED\n";

    gioInit();
    canInit();
    sciInit();
    sciSetBaudrate( sciREG, 115200U );
    sciSend( sciREG, ( uint32 ) strlen( pcBootMsg ), ( uint8 * ) pcBootMsg );

    memset( &xTelemetryData, 0, sizeof( xTelemetryData ) );
    xTelemetryMutex = xSemaphoreCreateMutexStatic( &xTelemetryMutexBuffer );
    configASSERT( xTelemetryMutex != NULL );

    init_drivers();

    xTaskCreateStatic( vCanRxTask, "CanRx", STACK_CAN_RX, NULL,
                        PRIO_CAN_RX, xCanRxStack, &xCanRxTaskBuffer );

    xTaskCreateStatic( vDriversTask, "Drivers", STACK_DRIVERS, NULL,
                        PRIO_DRIVERS, xDriversStack, &xDriversTaskBuffer );

    xTaskCreateStatic( vTelemetryTask, "Telemetry", STACK_TELEMETRY, NULL,
                        PRIO_TELEMETRY, xTelemetryStack, &xTelemetryTaskBuffer );

    vTaskStartScheduler();

    /* Only reached if there wasn't enough memory for the idle/timer task. */
    for( ;; )
    {
    }
}
