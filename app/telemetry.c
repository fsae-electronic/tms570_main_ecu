/*
 * Telemetry: owns the shared snapshot (xTelemetryData + xTelemetryMutex)
 * and the task that sends it over SCI/UART (XBee, 115200 8N1) as v3
 * fast/slow frames. The layout is in the "Wire frame (v3)" comment below
 * -- keep tools/decode_telemetry.py in sync with it.
 *
 * The task wakes every TELEMETRY_PERIOD_MS, takes a consistent copy of the
 * snapshot, and sends it. There is a placeholder for SAE compliance rules
 * between those two steps (not implemented yet, by design -- see the
 * comment in vTelemetryTask).
 */

#include "telemetry.h"

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"

#include "can.h"
#include "sci.h"

#include <string.h>

/* ---- Configuration ------------------------------------------------- */

#define STACK_TELEMETRY      384U
#define TELEMETRY_PERIOD_MS  10U

/* XBee configuration: 1 = SCITX (pin 39) / SCIRX (pin 38) stay high-Z from
 * boot and nothing is ever sent, so the XBee can be configured through its
 * own UART without the micro driving the lines. 0 = normal UART telemetry.
 * Flash with 1 to configure the XBee, then back to 0. */
#define XBEE_CONFIG_MODE      0

/* ---- Shared state ---------------------------------------------------*/

TelemetrySnapshot_t xTelemetryData;
SemaphoreHandle_t xTelemetryMutex;
static bool xTelemetryEnabled = true; /* cmd 0xD / 0xE on 0x600 */

static StaticSemaphore_t xTelemetryMutexBuffer;

static StaticTask_t xTelemetryTaskBuffer;
static StackType_t xTelemetryStack[ STACK_TELEMETRY ];

/* ---- Wire frame (v3) --------------------------------------------------
 *
 * Every multi-byte field is little-endian, matching the CAN payloads
 * themselves (see telemetry.h), and is serialized field by field (this CPU
 * is big-endian, so never memcpy a struct). Keep tools/decode_telemetry.py
 * in lockstep with this. Full spec: "Frame de telemetria v3".
 *
 * Common header, both frame types:
 *
 *   offset  size  field
 *   0       2     sync (0xAA 0x55)
 *   2       1     version (3)
 *   3       1     type (1 fast, 2 slow)
 *   4       1     seq, per type, wraps 255 -> 0
 *   5       1     payload_len (bytes between header and CRC)
 *   6       4     ecu_time_ms, when the snapshot was taken
 *   10      n     payload
 *   10+n    2     crc16, CRC-16/CCITT-FALSE over bytes 2 .. 9+n
 *
 * Fast frame (type 1, 130 bytes, every TELEMETRY_PERIOD_MS):
 *
 *   10   u32 valid_mask (TELEMETRY_VALID_*: received at least once)
 *   14   u32 fresh_mask (same bits: received within TELEMETRY_FRESH_MS)
 *   18   u16 ecu_state (ECU_STATE_*)
 *   20   u16 torque_limit_reason (TORQUE_LIMIT_*)
 *   22   u16 ts_state                 (no source yet: 0)
 *   24   u8  calibration_type
 *   25   u8  sdc_open_segment         (no source yet: 0)
 *   26   u16 tps1, tps2 (0.1 %)
 *   30   u16 front_brake, rear_brake (PSI)
 *   34   i16 steering_angle           (raw 0x502 value until calibrated)
 *   36   u16 wheel_speed_fl/fr/rl/rr (0.1 wheel rpm; rears = motor / GEAR_RATIO)
 *   44   i16 torque_request, torque_cmd_l, torque_cmd_r (per mille of rated)
 *   50   14  BMS block                (no source yet: 0)
 *   64   i16 dc_current_l/r, ac_current_l/r (raw 0x503 values)
 *   72   10  IMU, coolant, reserved, LV voltage (no source yet: 0)
 *   82   23  driver[0] (left), then 105 driver[1] (right):
 *              +0 control_word +2 status_word +4 warnings +6 error
 *              +8 target_torque +10 torque_actual +12 current_actual
 *              (both not mapped in the drives' PDOs yet: 0)
 *              +14 motor_velocity(i32) +18 dc_voltage +20 driver_temp
 *              +21 motor_temp +22 heartbeat_state
 *   128  u16 crc16
 *
 * Slow frame (type 2, 60 bytes, every TELEMETRY_SLOW_EVERY fast frames):
 *
 *   10   u8  fw_major, fw_minor, fw_patch, fw_dirty
 *   14   u32 git_hash
 *   18   u32 uptime_s
 *   22   u16 reset_count              (not stored yet: 0)
 *   24   u8  last_reset_cause         (startup clears SYSESR: 0 unknown)
 *   25   u8  cpu_load (%, last second: time not spent in the idle task)
 *   26   u16 loop_overruns (drivers loop longer than DRIVERS_PERIOD_MS,
 *            since boot), loop_time_max (us, worst drivers loop, last second)
 *   30   u32 active_faults (ACTIVE_FAULT_*)
 *   34   u8  can_tx_err, can_rx_err (CAN1 TEC / REC)
 *   36   u16 can_busoff_count         (not counted: 0)
 *   38   u32 motor_rated_current_l/r (mA, TXPDO3)
 *   46   u32 motor_rated_torque_l/r   (not read yet: 0)
 *   54   u8  cell_v_min_id, cell_v_max_id, cell_temp_max_id, reserved (0)
 *   58   u16 crc16
 */
#define TELEMETRY_VERSION        3U
#define TELEMETRY_TYPE_FAST      1U
#define TELEMETRY_TYPE_SLOW      2U
#define TELEMETRY_HEADER_SIZE    10U
#define TELEMETRY_FAST_SIZE      130U
#define TELEMETRY_SLOW_SIZE      60U
#define TELEMETRY_SLOW_EVERY     ( 1000U / TELEMETRY_PERIOD_MS ) /* 1 Hz */

/* fresh_mask window. TXPDO1 only comes every 500 ms, so it gets a longer
 * one or its bit would flicker even with the drive healthy. */
#define TELEMETRY_FRESH_MS        200U
#define TELEMETRY_FRESH_TXPDO1_MS 1000U

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 0U
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR 3U
#endif
#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH 0U
#endif
#ifndef FW_GIT_HASH
#define FW_GIT_HASH 0U /* passed by build.bat as make GIT_HASH=... */
#endif
#ifndef FW_DIRTY
#define FW_DIRTY 0U
#endif

#define TELEMETRY_VALID_TPS        ( 1UL << 0 )
#define TELEMETRY_VALID_FRONT      ( 1UL << 1 )
#define TELEMETRY_VALID_CURRENT    ( 1UL << 2 )
#define TELEMETRY_VALID_REAR       ( 1UL << 3 )
#define TELEMETRY_VALID_CAL        ( 1UL << 4 )
#define TELEMETRY_VALID_RXPDO1( n )    ( 1UL << ( 5U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO1( n )    ( 1UL << ( 6U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO2( n )    ( 1UL << ( 7U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_TXPDO3( n )    ( 1UL << ( 8U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_HEARTBEAT( n ) ( 1UL << ( 9U + ( 5U * ( n ) ) ) )
#define TELEMETRY_VALID_BMS        ( 1UL << 15 )
#define TELEMETRY_VALID_REAR_WHEEL ( 1UL << 16 ) /* both TXPDO2 */
#define TELEMETRY_VALID_IMU        ( 1UL << 17 )
#define TELEMETRY_VALID_COOLANT    ( 1UL << 18 )
#define TELEMETRY_VALID_LV         ( 1UL << 19 )
#define TELEMETRY_VALID_TS         ( 1UL << 20 )

#define ECU_STATE_MODE_RACE        ( 1U << 0 )  /* bits 0-1: 0 normal, 1 race */
#define ECU_STATE_DRIVE_ENABLED    ( 1U << 2 )
#define ECU_STATE_READY_TO_DRIVE   ( 1U << 3 )  /* handled by the on-board ECU: never set here */
#define ECU_STATE_TC_ENABLED       ( 1U << 4 )
#define ECU_STATE_TC_CUT_L         ( 1U << 5 )
#define ECU_STATE_TC_CUT_R         ( 1U << 6 )
#define ECU_STATE_APPS_IMPLAUSIBLE ( 1U << 7 )  /* 0x401 fault or 0x501 timeout */
#define ECU_STATE_APPS_BRAKE       ( 1U << 8 )  /* EV.4.7 */
#define ECU_STATE_BRAKE_PRESSED    ( 1U << 9 )
#define ECU_STATE_SAFE_MODE        ( 1U << 10 ) /* torque forced to 0 by a fault */
#define ECU_STATE_DRIVER_FAULT     ( 1U << 11 ) /* a drive faulted or dropped out in DRIVE */
#define ECU_STATE_BRAKE_LIGHT      ( 1U << 12 ) /* brake light output on */
#define ECU_STATE_RTDS             ( 1U << 13 ) /* Ready to Drive sound output on */

#define TORQUE_LIMIT_DRIVE_DISABLED  ( 1U << 0 )
#define TORQUE_LIMIT_APPS            ( 1U << 1 )
#define TORQUE_LIMIT_APPS_BRAKE      ( 1U << 2 )
#define TORQUE_LIMIT_TRACTION        ( 1U << 3 )
#define TORQUE_LIMIT_POWER           ( 1U << 4 ) /* race map DC current fall-off */
#define TORQUE_LIMIT_DRIVER_DISABLED ( 1U << 11 )

/* active_faults (slow frame): bits 0-2 are SENSOR_TIMEOUT_*. */
#define ACTIVE_FAULT_SENSOR_IMPLAUSIBLE ( 1UL << 3 ) /* 0x401 latched */
#define ACTIVE_FAULT_APPS_BRAKE         ( 1UL << 4 ) /* EV.4.7 */
#define ACTIVE_FAULT_DRIVER_LEFT        ( 1UL << 5 ) /* drive 1 faulted / dropped out */
#define ACTIVE_FAULT_DRIVER_RIGHT       ( 1UL << 6 ) /* drive 2 faulted / dropped out */

/* CiA 402 status word: "Operation enabled" under mask 0x6F. */
#define STATUSWORD_STATE_MASK     0x006FU
#define STATUSWORD_OP_ENABLED     0x0027U

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

static void putZeros( uint8_t *pxFrame, uint32_t *pIdx, uint32_t ulCount )
{
    memset( &pxFrame[ *pIdx ], 0, ulCount );
    *pIdx += ulCount;
}

/* Saturate to the u16 range, for fields that get scaled up. */
static uint16_t usSat( int32_t v )
{
    if( v < 0 )
    {
        return 0U;
    }
    if( v > 0xFFFF )
    {
        return 0xFFFFU;
    }
    return ( uint16_t ) v;
}

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final
 * xor. crc16("123456789") == 0x29B1. */
static uint16_t usCrc16( const uint8_t *pxData, uint32_t ulLen )
{
    uint16_t usCrc = 0xFFFFU;
    uint32_t i;

    while( ulLen-- > 0U )
    {
        usCrc ^= ( uint16_t ) ( ( uint16_t ) *pxData++ << 8 );
        for( i = 0U; i < 8U; i++ )
        {
            usCrc = ( ( usCrc & 0x8000U ) != 0U ) ? ( uint16_t ) ( ( usCrc << 1 ) ^ 0x1021U )
                                                  : ( uint16_t ) ( usCrc << 1 );
        }
    }

    return usCrc;
}

static void vPutHeader( uint8_t *pxFrame, uint32_t *pIdx, uint8_t ucType, uint8_t ucSeq,
                        uint8_t ucPayloadLen, uint32_t ulTimeMs )
{
    putU8( pxFrame, pIdx, 0xAAU );
    putU8( pxFrame, pIdx, 0x55U );
    putU8( pxFrame, pIdx, TELEMETRY_VERSION );
    putU8( pxFrame, pIdx, ucType );
    putU8( pxFrame, pIdx, ucSeq );
    putU8( pxFrame, pIdx, ucPayloadLen );
    putU32( pxFrame, pIdx, ulTimeMs );
}

/* CRC over version .. end of payload, appended at *pIdx. */
static void vPutCrc( uint8_t *pxFrame, uint32_t *pIdx )
{
    putU16( pxFrame, pIdx, usCrc16( &pxFrame[ 2 ], *pIdx - 2U ) );
}

static bool bFresh( bool bValid, TickType_t xRxTick, TickType_t xNow, uint32_t ulWindowMs )
{
    return bValid && ( ( xNow - xRxTick ) <= pdMS_TO_TICKS( ulWindowMs ) );
}

static uint32_t ulValidMask( const TelemetrySnapshot_t *pxSnapshot )
{
    uint32_t ulMask = 0U;
    uint32_t n;

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
    if( pxSnapshot->driver[ 0 ].txpdo2_valid && pxSnapshot->driver[ 1 ].txpdo2_valid )
    {
        ulMask |= TELEMETRY_VALID_REAR_WHEEL;
    }

    return ulMask;
}

static uint32_t ulFreshMask( const TelemetrySnapshot_t *pxSnapshot, TickType_t xNow )
{
    uint32_t ulMask = 0U;
    uint32_t n;
    bool bRearWheels = true;

    if( bFresh( pxSnapshot->sensors_valid, pxSnapshot->sensors_rx_tick, xNow, TELEMETRY_FRESH_MS ) )         { ulMask |= TELEMETRY_VALID_TPS; }
    if( bFresh( pxSnapshot->front_axle_valid, pxSnapshot->front_axle_rx_tick, xNow, TELEMETRY_FRESH_MS ) )   { ulMask |= TELEMETRY_VALID_FRONT; }
    if( bFresh( pxSnapshot->current_valid, pxSnapshot->current_rx_tick, xNow, TELEMETRY_FRESH_MS ) )         { ulMask |= TELEMETRY_VALID_CURRENT; }
    if( bFresh( pxSnapshot->rear_valid, pxSnapshot->rear_rx_tick, xNow, TELEMETRY_FRESH_MS ) )               { ulMask |= TELEMETRY_VALID_REAR; }
    if( bFresh( pxSnapshot->calibration_valid, pxSnapshot->calibration_rx_tick, xNow, TELEMETRY_FRESH_MS ) ) { ulMask |= TELEMETRY_VALID_CAL; }
    for( n = 0U; n < 2U; n++ )
    {
        const DriverNode_t *pxNode = &pxSnapshot->driver[ n ];

        if( bFresh( pxNode->rxpdo1_valid, pxNode->rxpdo1_rx_tick, xNow, TELEMETRY_FRESH_MS ) )        { ulMask |= TELEMETRY_VALID_RXPDO1( n ); }
        if( bFresh( pxNode->txpdo1_valid, pxNode->txpdo1_rx_tick, xNow, TELEMETRY_FRESH_TXPDO1_MS ) ) { ulMask |= TELEMETRY_VALID_TXPDO1( n ); }
        if( bFresh( pxNode->txpdo2_valid, pxNode->txpdo2_rx_tick, xNow, TELEMETRY_FRESH_MS ) )        { ulMask |= TELEMETRY_VALID_TXPDO2( n ); }
        else                                                                                           { bRearWheels = false; }
        if( bFresh( pxNode->txpdo3_valid, pxNode->txpdo3_rx_tick, xNow, TELEMETRY_FRESH_MS ) )        { ulMask |= TELEMETRY_VALID_TXPDO3( n ); }
        if( bFresh( pxNode->heartbeat_valid, pxNode->heartbeat_rx_tick, xNow, TELEMETRY_FRESH_MS ) )  { ulMask |= TELEMETRY_VALID_HEARTBEAT( n ); }
    }
    if( bRearWheels )
    {
        ulMask |= TELEMETRY_VALID_REAR_WHEEL;
    }

    return ulMask;
}

static bool bAppsImplausible( const TelemetrySnapshot_t *pxSnapshot )
{
    return pxSnapshot->sensor_fault || ( ( pxSnapshot->sensor_timeouts & SENSOR_TIMEOUT_TPS ) != 0U );
}

static bool bDriverNotEnabled( const DriverNode_t *pxNode )
{
    return !pxNode->txpdo1_valid ||
           ( ( pxNode->status_word & STATUSWORD_STATE_MASK ) != STATUSWORD_OP_ENABLED );
}

static uint16_t usEcuState( const TelemetrySnapshot_t *pxSnapshot )
{
    uint16_t usState = 0U;

    if( pxSnapshot->race_mode )          { usState |= ECU_STATE_MODE_RACE; }
    if( pxSnapshot->drive_enabled )      { usState |= ECU_STATE_DRIVE_ENABLED; }
    if( pxSnapshot->traction_on )        { usState |= ECU_STATE_TC_ENABLED; }
    if( pxSnapshot->tc_cutting[ 0 ] )    { usState |= ECU_STATE_TC_CUT_L; }
    if( pxSnapshot->tc_cutting[ 1 ] )    { usState |= ECU_STATE_TC_CUT_R; }
    if( bAppsImplausible( pxSnapshot ) ) { usState |= ECU_STATE_APPS_IMPLAUSIBLE; }
    if( pxSnapshot->apps_brake_fault )   { usState |= ECU_STATE_APPS_BRAKE; }
    if( pxSnapshot->brake_engaged )      { usState |= ECU_STATE_BRAKE_PRESSED; }
    if( pxSnapshot->driver_faults != 0U ) { usState |= ECU_STATE_DRIVER_FAULT; }
    if( pxSnapshot->brake_light )        { usState |= ECU_STATE_BRAKE_LIGHT; }
    if( pxSnapshot->rtds_on )            { usState |= ECU_STATE_RTDS; }
    if( pxSnapshot->sensor_fault || pxSnapshot->apps_brake_fault || ( pxSnapshot->sensor_timeouts != 0U ) ||
        ( pxSnapshot->driver_faults != 0U ) )
    {
        usState |= ECU_STATE_SAFE_MODE;
    }

    return usState;
}

static uint16_t usTorqueLimitReason( const TelemetrySnapshot_t *pxSnapshot )
{
    uint16_t usReason = 0U;

    if( !pxSnapshot->drive_enabled )     { usReason |= TORQUE_LIMIT_DRIVE_DISABLED; }
    if( bAppsImplausible( pxSnapshot ) ) { usReason |= TORQUE_LIMIT_APPS; }
    if( pxSnapshot->apps_brake_fault )   { usReason |= TORQUE_LIMIT_APPS_BRAKE; }
    if( pxSnapshot->tc_cutting[ 0 ] || pxSnapshot->tc_cutting[ 1 ] ) { usReason |= TORQUE_LIMIT_TRACTION; }
    if( pxSnapshot->race_map_limiting )  { usReason |= TORQUE_LIMIT_POWER; }
    if( pxSnapshot->drive_enabled &&
        ( bDriverNotEnabled( &pxSnapshot->driver[ 0 ] ) || bDriverNotEnabled( &pxSnapshot->driver[ 1 ] ) ) )
    {
        usReason |= TORQUE_LIMIT_DRIVER_DISABLED;
    }

    return usReason;
}

static void vPackFastFrame( const TelemetrySnapshot_t *pxSnapshot, TickType_t xNow, uint8_t ucSeq,
                            uint8_t *pxFrame )
{
    uint32_t idx = 0U;
    uint32_t n;
    int16_t s16Request;

    vPutHeader( pxFrame, &idx, TELEMETRY_TYPE_FAST, ucSeq,
                ( uint8_t ) ( TELEMETRY_FAST_SIZE - TELEMETRY_HEADER_SIZE - 2U ),
                ( uint32_t ) ( xNow * portTICK_PERIOD_MS ) );

    /* State and validity */
    putU32( pxFrame, &idx, ulValidMask( pxSnapshot ) );
    putU32( pxFrame, &idx, ulFreshMask( pxSnapshot, xNow ) );
    putU16( pxFrame, &idx, usEcuState( pxSnapshot ) );
    putU16( pxFrame, &idx, usTorqueLimitReason( pxSnapshot ) );
    putU16( pxFrame, &idx, 0U ); /* ts_state */
    putU8( pxFrame, &idx, pxSnapshot->calibration_type );
    putU8( pxFrame, &idx, 0U );  /* sdc_open_segment */

    /* Pedals and chassis: TPS is 0-100 % on 0x501 -> 0.1 %; wheel speeds
     * are wheel rpm -> 0.1 rpm. */
    putU16( pxFrame, &idx, usSat( ( int32_t ) pxSnapshot->sensors.tps1 * 10 ) );
    putU16( pxFrame, &idx, usSat( ( int32_t ) pxSnapshot->sensors.tps2 * 10 ) );
    putU16( pxFrame, &idx, pxSnapshot->front_axle.front_brake );
    putU16( pxFrame, &idx, pxSnapshot->rear.rear_brake );
    putU16( pxFrame, &idx, pxSnapshot->front_axle.steering_angle ); /* TODO: 0.1 deg once the steering calibration is known */
    putU16( pxFrame, &idx, usSat( ( int32_t ) pxSnapshot->front_axle.wheel_speed_fl * 10 ) );
    putU16( pxFrame, &idx, usSat( ( int32_t ) pxSnapshot->front_axle.wheel_speed_fr * 10 ) );
    putU16( pxFrame, &idx, usSat( pxSnapshot->rear.wheel_speed_rl * 10 ) );
    putU16( pxFrame, &idx, usSat( pxSnapshot->rear.wheel_speed_rr * 10 ) );

    /* Torque: one request field; both maps agree except in race mode. */
    s16Request = ( pxSnapshot->torque_request[ 0 ] > pxSnapshot->torque_request[ 1 ] ) ? pxSnapshot->torque_request[ 0 ]
                                                                                       : pxSnapshot->torque_request[ 1 ];
    putU16( pxFrame, &idx, ( uint16_t ) s16Request );
    putU16( pxFrame, &idx, ( uint16_t ) pxSnapshot->torque_cmd[ 0 ] );
    putU16( pxFrame, &idx, ( uint16_t ) pxSnapshot->torque_cmd[ 1 ] );

    putZeros( pxFrame, &idx, 14U ); /* BMS: pack V/I, SoC, cells, limit, faults */

    /* Currents from 0x503, 1 = left, 2 = right. */
    putU16( pxFrame, &idx, pxSnapshot->current.dc_current_1 );
    putU16( pxFrame, &idx, pxSnapshot->current.dc_current_2 );
    putU16( pxFrame, &idx, pxSnapshot->current.ac_current_1 );
    putU16( pxFrame, &idx, pxSnapshot->current.ac_current_2 );

    putZeros( pxFrame, &idx, 10U ); /* accel_x/y, yaw_rate, coolant, reserved, lv_voltage */

    for( n = 0U; n < 2U; n++ )
    {
        const DriverNode_t *pxNode = &pxSnapshot->driver[ n ];

        putU16( pxFrame, &idx, pxNode->control_word );
        putU16( pxFrame, &idx, pxNode->status_word );
        putU16( pxFrame, &idx, pxNode->warnings );
        putU16( pxFrame, &idx, pxNode->error );
        putU16( pxFrame, &idx, ( uint16_t ) pxNode->target_torque );
        putU16( pxFrame, &idx, 0U ); /* torque_actual 0x6077 */
        putU16( pxFrame, &idx, 0U ); /* current_actual 0x6078 */
        putU32( pxFrame, &idx, ( uint32_t ) pxNode->motor_velocity );
        putU16( pxFrame, &idx, pxNode->driver_dc_voltage );
        putU8( pxFrame, &idx, pxNode->driver_temp );
        putU8( pxFrame, &idx, pxNode->motor_temp );
        putU8( pxFrame, &idx, pxNode->heartbeat_state );
    }

    vPutCrc( pxFrame, &idx );
    configASSERT( idx == TELEMETRY_FAST_SIZE );
}

/* CPU load since the previous call, in %: whatever the idle task didn't
 * get. Both counters are RUN_TIME_COUNTER_HZ counts that wrap, so only
 * their differences are used (fine for calls < ~107 s apart). */
static uint8_t ucCpuLoad( void )
{
    static uint32_t ulLastIdle = 0U;
    static uint32_t ulLastTotal = 0U;
    TaskStatus_t xIdle;
    uint32_t ulTotal = portGET_RUN_TIME_COUNTER_VALUE();
    uint32_t ulDIdle;
    uint32_t ulDTotal;

    vTaskGetInfo( xTaskGetIdleTaskHandle(), &xIdle, pdFALSE, eReady );

    ulDIdle = xIdle.ulRunTimeCounter - ulLastIdle;
    ulDTotal = ulTotal - ulLastTotal;
    ulLastIdle = xIdle.ulRunTimeCounter;
    ulLastTotal = ulTotal;

    if( ( ulDTotal == 0U ) || ( ulDIdle >= ulDTotal ) )
    {
        return 0U;
    }

    return ( uint8_t ) ( 100U - ( uint32_t ) ( ( 100ULL * ulDIdle ) / ulDTotal ) );
}

static void vPackSlowFrame( const TelemetrySnapshot_t *pxSnapshot, TickType_t xNow, uint8_t ucSeq,
                            uint8_t ucLoad, uint8_t *pxFrame )
{
    uint32_t idx = 0U;
    uint32_t ulFaults = pxSnapshot->sensor_timeouts;
    uint32_t ulErrCnt = canREG1->EERC; /* TEC in 7:0, REC in 14:8 */

    if( pxSnapshot->sensor_fault )     { ulFaults |= ACTIVE_FAULT_SENSOR_IMPLAUSIBLE; }
    if( pxSnapshot->apps_brake_fault ) { ulFaults |= ACTIVE_FAULT_APPS_BRAKE; }
    if( ( pxSnapshot->driver_faults & DRIVER_FAULT_LEFT ) != 0U )  { ulFaults |= ACTIVE_FAULT_DRIVER_LEFT; }
    if( ( pxSnapshot->driver_faults & DRIVER_FAULT_RIGHT ) != 0U ) { ulFaults |= ACTIVE_FAULT_DRIVER_RIGHT; }

    vPutHeader( pxFrame, &idx, TELEMETRY_TYPE_SLOW, ucSeq,
                ( uint8_t ) ( TELEMETRY_SLOW_SIZE - TELEMETRY_HEADER_SIZE - 2U ),
                ( uint32_t ) ( xNow * portTICK_PERIOD_MS ) );

    putU8( pxFrame, &idx, FW_VERSION_MAJOR );
    putU8( pxFrame, &idx, FW_VERSION_MINOR );
    putU8( pxFrame, &idx, FW_VERSION_PATCH );
    putU8( pxFrame, &idx, FW_DIRTY );
    putU32( pxFrame, &idx, FW_GIT_HASH );
    putU32( pxFrame, &idx, ( uint32_t ) ( ( xNow * portTICK_PERIOD_MS ) / 1000U ) );
    putU16( pxFrame, &idx, 0U ); /* reset_count */
    putU8( pxFrame, &idx, 0U );  /* last_reset_cause */
    putU8( pxFrame, &idx, ucLoad );
    putU16( pxFrame, &idx, pxSnapshot->loop_overruns );
    putU16( pxFrame, &idx, pxSnapshot->loop_time_max_us );
    putU32( pxFrame, &idx, ulFaults );
    putU8( pxFrame, &idx, ( uint8_t ) ( ulErrCnt & 0xFFU ) );
    putU8( pxFrame, &idx, ( uint8_t ) ( ( ulErrCnt >> 8 ) & 0x7FU ) );
    putU16( pxFrame, &idx, 0U ); /* can_busoff_count */
    putU32( pxFrame, &idx, pxSnapshot->driver[ 0 ].motor_rated_current );
    putU32( pxFrame, &idx, pxSnapshot->driver[ 1 ].motor_rated_current );
    putU32( pxFrame, &idx, 0U ); /* motor_rated_torque_l 0x6076 */
    putU32( pxFrame, &idx, 0U ); /* motor_rated_torque_r */
    putZeros( pxFrame, &idx, 4U ); /* cell ids, reserved */

    vPutCrc( pxFrame, &idx );
    configASSERT( idx == TELEMETRY_SLOW_SIZE );
}

#if XBEE_CONFIG_MODE
/* SCITX / SCIRX pins to high-Z (GIO input, pull disabled), so the micro
 * neither drives nor loads the XBee lines. */
#define SCI_PINS ( ( uint32 ) ( 1U << 2U ) | ( uint32 ) ( 1U << 1U ) ) /* bit 2 TX, bit 1 RX */

static void vSciPinsHighZ( void )
{
    sciREG->PIO1 &= ~SCI_PINS; /* GIO direction: input */
    sciREG->PIO7 |= SCI_PINS;  /* pull-up/down disabled */
    sciREG->PIO0 &= ~SCI_PINS; /* release the pins from the SCI */
}
#endif

static portTASK_FUNCTION( vTelemetryTask, pvParameters )
{
    TelemetrySnapshot_t xSnapshot;
    uint8_t xFrame[ TELEMETRY_FAST_SIZE ];
    TickType_t xNow;
    bool bTelemetryEnabled;
    bool bSlowDue;
    uint8_t ucLoad = 0U;
    uint8_t ucFastSeq = 0U;
    uint8_t ucSlowSeq = 0U;
    uint32_t ulSlowCountdown = 0U; /* first slow frame right away */
    ( void ) pvParameters;

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( TELEMETRY_PERIOD_MS ) );

        /* The slow frame's 1 s window: CPU load and worst loop time are
         * measured from one slow frame to the next, even while the stream
         * is off. */
        bSlowDue = ( ulSlowCountdown == 0U );
        if( bSlowDue )
        {
            ulSlowCountdown = TELEMETRY_SLOW_EVERY;
            ucLoad = ucCpuLoad();
        }
        ulSlowCountdown--;

        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xSnapshot = xTelemetryData;
        if( bSlowDue )
        {
            xTelemetryData.loop_time_max_us = 0U;
        }
        bTelemetryEnabled = xTelemetryEnabled;
        xNow = xTaskGetTickCount();
        xSemaphoreGive( xTelemetryMutex );

        /* XBee config mode: pins are high-Z, never send. */
        if( ( XBEE_CONFIG_MODE != 0 ) || !bTelemetryEnabled )
        {
            continue;
        }

        /* TODO: SAE compliance rules go here, between reading the
         * snapshot and sending it -- deliberately not implemented yet.
         * For now this is a straight pass-through. */

        vPackFastFrame( &xSnapshot, xNow, ucFastSeq++, xFrame );
        sciSend( sciREG, TELEMETRY_FAST_SIZE, xFrame );

        if( bSlowDue )
        {
            vPackSlowFrame( &xSnapshot, xNow, ucSlowSeq++, ucLoad, xFrame );
            sciSend( sciREG, TELEMETRY_SLOW_SIZE, xFrame );
        }
    }
}

/* ---- Public API -------------------------------------------------------*/

void telemetry_init( UBaseType_t uxPriority )
{
    const char *pcBootMsg = "TMS570 MAIN ECU (rewrite) INITIALIZED\n";

#if XBEE_CONFIG_MODE
    ( void ) pcBootMsg;
    vSciPinsHighZ();
#else
    sciSend( sciREG, ( uint32 ) strlen( pcBootMsg ), ( uint8 * ) pcBootMsg );
#endif

    memset( &xTelemetryData, 0, sizeof( xTelemetryData ) );
    xTelemetryMutex = xSemaphoreCreateMutexStatic( &xTelemetryMutexBuffer );
    configASSERT( xTelemetryMutex != NULL );

    xTaskCreateStatic( vTelemetryTask, "Telemetry", STACK_TELEMETRY, NULL,
                        uxPriority, xTelemetryStack, &xTelemetryTaskBuffer );
}

void telemetry_set_enabled( bool bEnabled )
{
    xTelemetryEnabled = bEnabled;
}
