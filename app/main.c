/*
 * Main ECU application -- clean rewrite.
 *
 * Pipeline: CAN RX task polls every mailbox this ECU cares about (see
 * telemetry.h / the "Mensajes CAN" spec / can-logger/cansniff/messages.py
 * for the catalogue) and updates the mutex-protected snapshot
 * (xTelemetryData); the drivers task turns it into torque requests, and
 * the telemetry task (telemetry.c) sends it out over SCI/UART.
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
#include "charger.h"

#include "telemetry.h"
#include "traction.h"

#include <math.h>
#include <stdlib.h>

/* ---- Configuration ------------------------------------------------- */

#define PRIO_CAN_RX       3U
#define PRIO_DRIVERS      2U
#define PRIO_TELEMETRY    1U
#define PRIO_CHARGER      1U

#define STACK_CAN_RX       256U
#define STACK_DRIVERS      256U

#define CAN_RX_PERIOD_MS      10U
#define DRIVERS_PERIOD_MS     10U

/* CAN ID 0x600 B0 command ids, same values as the on-board ECU's
 * calibration_cmd_id_t. 1-6 are sensor calibrations for the adc2can
 * boards, so this ECU only acts on the ones below. */
#define CALIBRATION_CMD_DRIVE_ENABLE_ON   0x7U
#define CALIBRATION_CMD_DRIVE_ENABLE_OFF  0x8U
#define CALIBRATION_CMD_TRACTION_ON       0x9U
#define CALIBRATION_CMD_TRACTION_OFF      0xAU
#define CALIBRATION_CMD_MODE_NORMAL       0xBU
#define CALIBRATION_CMD_MODE_RACE         0xCU
#define CALIBRATION_CMD_TELEMETRY_ENABLE  0xDU
#define CALIBRATION_CMD_TELEMETRY_DISABLE 0xEU

/* TPS arrives on 0x501 as 0-100 % (adc2can_front), but the drives take
 * target torque in 0-1000 per mille of rated torque (1000 = motor rated
 * current), and the race map is written for that 0-1000 scale too.
 * TESTING: left at 1 so the pedal only spans 0-100 (1/10 of the range);
 * set to 10 for full torque. */
#define TPS_TO_TORQUE           10U

/* EV.4.7 APPS / Brake Pedal Plausibility Check: brake engaged (front or
 * rear pressure above BRAKE_ENGAGED_PSI) while either APPS reads more than
 * APPS_BRAKE_TRIP_PCT stops the motors; they stay stopped until both APPS
 * read less than APPS_BRAKE_RESET_PCT, with or without brake. TPS is the
 * raw 0-100 % pedal travel from 0x501. */
#define BRAKE_ENGAGED_PSI       50U  /* same as the on-board ECU's DRIVE_ENABLE_MIN_BRAKE_PSI */
#define APPS_BRAKE_TRIP_PCT     25U
#define APPS_BRAKE_RESET_PCT    5U

/* T.4.2.11 / T.4.3.5: APPS and BSE arrive over CAN, so losing their frames
 * is a sensor failure. If 0x501 (TPS), 0x502 (front brake) or 0x504 (rear
 * brake) hasn't arrived for more than SENSOR_CAN_TIMEOUT_MS -- or never
 * arrived -- drive is forced off like an implausibility; it takes a new
 * drive-on (cmd 7) once the frames are back. The adc2can boards send
 * every 10 ms. */
#define SENSOR_CAN_TIMEOUT_MS   100U /* SENSOR_TIMEOUT_* bits: telemetry.h */

/* 0x403 to the on-board ECU: B0 EV.4.7 active, B1 SENSOR_TIMEOUT_* bits.
 * Sent whenever either changes, and repeated every
 * MAIN_ECU_ERRORS_PERIOD_MS while any is active so a dashboard that missed
 * the edge (or booted late) still shows it. */
#define MAIN_ECU_ERRORS_MAILBOX     canMESSAGE_BOX28
#define MAIN_ECU_ERRORS_PERIOD_MS   100U

/* Motor turns per rear wheel turn: rear wheel speed = motor_velocity /
 * GEAR_RATIO, in wheel RPM like the fronts (needed for the slip ratio). */
#define GEAR_RATIO              4

/* Low-pass on each TXPDO2 velocity sample (every 10 ms): alpha 0.25 is a
 * ~30 ms time constant. The raw value is noisy enough to make the race map
 * request and the TC slip jump around from one cycle to the next. */
#define VELOCITY_FILTER_ALPHA   0.35f

/* Race-mode torque map limits */
#define MAX_DC_CURRENT			((int32_t)75) //A, battery DC limit per driver
#define KV_EMRAX				((float)22.00) //RPM/V

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

/* ---- Task storage ---------------------------------------------------*/

static StaticTask_t xCanRxTaskBuffer;
static StackType_t xCanRxStack[ STACK_CAN_RX ];

static StaticTask_t xDriversTaskBuffer;
static StackType_t xDriversStack[ STACK_DRIVERS ];

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
        xTelemetryData.driver[ node ].txpdo1_rx_tick = xTaskGetTickCount();
        xSemaphoreGive( xTelemetryMutex );
    }

    /* TXPDO2 (10 ms): B0-3 0x606C velocity actual value (int32),
     * B4-5 0x6079 DC link circuit voltage (uint16). */
    if( canGetData( canREG1, boxTxPdo2, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].motor_velocity     = (-1)*(leI32( &raw[ 0 ] ));
        if( !xTelemetryData.driver[ node ].txpdo2_valid )
        {
            /* First sample: seed the filter instead of ramping up from 0. */
            xTelemetryData.driver[ node ].motor_velocity_f = ( float ) xTelemetryData.driver[ node ].motor_velocity;
        }
        else
        {
            xTelemetryData.driver[ node ].motor_velocity_f +=
                VELOCITY_FILTER_ALPHA * ( ( float ) xTelemetryData.driver[ node ].motor_velocity -
                                          xTelemetryData.driver[ node ].motor_velocity_f );
        }
        if( node == 0U )
        {
            xTelemetryData.rear.wheel_speed_rl = ( int32_t ) ( xTelemetryData.driver[ node ].motor_velocity_f / GEAR_RATIO );
        }
        else
        {
            xTelemetryData.rear.wheel_speed_rr = ( int32_t ) ( xTelemetryData.driver[ node ].motor_velocity_f / GEAR_RATIO );
        }
        xTelemetryData.driver[ node ].driver_dc_voltage = leU16( &raw[ 4 ] );
        xTelemetryData.driver[ node ].txpdo2_valid = true;
        xTelemetryData.driver[ node ].txpdo2_rx_tick = xTaskGetTickCount();
        xSemaphoreGive( xTelemetryMutex );
    }

    /* TXPDO3 (100 ms): B0-3 0x6075 motor rated current (uint32, mA),
     * B4 0x2026 controller temperature, B5 0x2025 motor temperature. */
    if( canGetData( canREG1, boxTxPdo3, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].motor_rated_current = leU32( &raw[ 0 ] );
        xTelemetryData.driver[ node ].driver_temp         = raw[ 4 ];
        xTelemetryData.driver[ node ].motor_temp          = raw[ 5 ];
        xTelemetryData.driver[ node ].txpdo3_valid = true;
        xTelemetryData.driver[ node ].txpdo3_rx_tick = xTaskGetTickCount();
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxRxPdo1, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].control_word  = leU16( &raw[ 0 ] );
        xTelemetryData.driver[ node ].target_torque = leI16( &raw[ 2 ] );
        xTelemetryData.driver[ node ].rxpdo1_valid = true;
        xTelemetryData.driver[ node ].rxpdo1_rx_tick = xTaskGetTickCount();
        xSemaphoreGive( xTelemetryMutex );
    }

    if( canGetData( canREG1, boxHeartbeat, raw ) != 0U )
    {
        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
        xTelemetryData.driver[ node ].heartbeat_state = raw[ 0 ];
        xTelemetryData.driver[ node ].heartbeat_valid = true;
        xTelemetryData.driver[ node ].heartbeat_rx_tick = xTaskGetTickCount();
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
            xTelemetryData.sensors.tps1 = leU16(&raw[ 0 ]);
            xTelemetryData.sensors.tps2 = leU16(&raw[ 2 ]);
            xTelemetryData.sensors_valid = true;
            xTelemetryData.sensors_rx_tick = xTaskGetTickCount();
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
            xTelemetryData.front_axle_rx_tick = xTaskGetTickCount();
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
            xTelemetryData.current_rx_tick = xTaskGetTickCount();
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX13, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.rear.rear_brake = leU16( &raw[ 0 ] );
            xTelemetryData.rear_valid = true;
            xTelemetryData.rear_rx_tick = xTaskGetTickCount();
            xSemaphoreGive( xTelemetryMutex );
        }

        if( canGetData( canREG1, canMESSAGE_BOX24, raw ) != 0U )
        {
            xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
            xTelemetryData.calibration_type = raw[ 0 ];
            xTelemetryData.calibration_value = raw[ 1 ];
            xTelemetryData.calibration_valid = true;
            xTelemetryData.calibration_rx_tick = xTaskGetTickCount();
            if( raw[ 0 ] == CALIBRATION_CMD_DRIVE_ENABLE_ON )
            {
                xTelemetryData.drive_enabled = true;
                xTelemetryData.sensor_fault = false;
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_DRIVE_ENABLE_OFF )
            {
                xTelemetryData.drive_enabled = false;
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_TRACTION_ON )
            {
                xTelemetryData.traction_on = true;
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_TRACTION_OFF )
            {
                xTelemetryData.traction_on = false;
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_TELEMETRY_ENABLE )
            {
                telemetry_set_enabled( true );
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_TELEMETRY_DISABLE )
            {
                telemetry_set_enabled( false );
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_MODE_NORMAL )
            {
                xTelemetryData.race_mode = false;
            }
            else if( raw[ 0 ] == CALIBRATION_CMD_MODE_RACE )
            {
                xTelemetryData.race_mode = true;
            }
            xSemaphoreGive( xTelemetryMutex );
        }

        /* CAN ID 0x401: TPS / brake implausibility, detected by the
         * adc2can boards and sent once per fault. Stop the car by forcing
         * drive off, exactly as if cmd 8 had arrived -- it stays stopped
         * until the on-board ECU sends a new drive-on (cmd 7). Polled after
         * 0x600 so a fault wins over a drive-on in the same cycle. */
        if( canGetData( canREG1, canMESSAGE_BOX27, raw ) != 0U )
        {
            if( ( raw[ 0 ] != 0U ) || ( raw[ 1 ] != 0U ) )
            {
                xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
                xTelemetryData.drive_enabled = false;
                xTelemetryData.sensor_fault = true;
                xSemaphoreGive( xTelemetryMutex );
            }
        }

        vPollDriverNode( 0U, canMESSAGE_BOX14, canMESSAGE_BOX16, canMESSAGE_BOX18,
                          canMESSAGE_BOX20, canMESSAGE_BOX22 );
        vPollDriverNode( 1U, canMESSAGE_BOX15, canMESSAGE_BOX17, canMESSAGE_BOX19,
                          canMESSAGE_BOX21, canMESSAGE_BOX23 );

        vTaskDelay( pdMS_TO_TICKS( CAN_RX_PERIOD_MS ) );
    }
}

/* Normal-mode torque for one driver node: linear with the pedal, scaled so
 * full pedal asks for the torque whose phase current equals the battery DC
 * limit (at top speed I_phase_rms ~ I_dc):
 *     torque_max = 1000 * MAX_DC_CURRENT / motor_rated_current
 * e.g. 75 A DC with 200 A rated -> 375. motor_rated_current is the phase
 * current at torque 1000, in mA (TXPDO3); until it arrives there is no
 * current reference, so the request stays at 0. sensor_value and the result
 * are 0-1000. Call with xTelemetryMutex held. */
static int16_t s16NormalTorque( uint32_t sensor_value, const DriverNode_t *driver )
{
    uint32_t ulTorqueMax;

    if( driver->motor_rated_current == 0U )
    {
        return 0;
    }

    ulTorqueMax = ( 1000U * ( uint32_t ) MAX_DC_CURRENT * 1000U ) / driver->motor_rated_current;
    if( ulTorqueMax > 1000U )
    {
        ulTorqueMax = 1000U;
    }

    return ( int16_t ) ( ( sensor_value * ulTorqueMax ) / 1000U );
}

/* Race-mode torque map for one driver node: follows the pedal until the
 * motor goes past M * max_velocity (max_velocity = KV_EMRAX * DC link
 * voltage), then the request falls off as 1/velocity so the DC current
 * stays around MAX_DC_CURRENT. sensor_value and the result are 0-1000
 * (per mille of rated torque). Call with xTelemetryMutex held. */
static int16_t s16RaceTorqueMap( float sensor_value, const DriverNode_t *driver )
{
	int16_t tps_value;
	float velocity_value = fabsf(driver->motor_velocity_f);
	float max_velocity = (float) (KV_EMRAX*((float)(driver->driver_dc_voltage)/10.0f));

	// Calculate the factor
	float Mmin = (MAX_DC_CURRENT)/((float)(driver->motor_rated_current)/1000.0f);

	float M = Mmin;

	if(sensor_value > 10.0)
	{
		M  = Mmin *(1000.0f / sensor_value);
	}
	else
	{
		M = 100.0;
	}

	if(velocity_value > M * max_velocity)
	{
		//tps = Mmin * 1000.0 * (VEL_MAX / velocity)
		tps_value = (uint16_t)((Mmin * 1000.0f * max_velocity) / velocity_value);
	}
	else
	{
		tps_value = (uint16_t)sensor_value;
	}

	return tps_value;
}

/* EV.4.7: latches on brake + pedal > 25 %, releases on pedal < 5 % alone
 * (no manual action needed). Uses the higher of the two APPS so a single
 * sensor reading high is enough to trip and keep it tripped. Call with
 * xTelemetryMutex held. */
static bool bBrakeEngaged( const TelemetrySnapshot_t *pxData )
{
    return ( pxData->front_axle_valid && ( pxData->front_axle.front_brake > BRAKE_ENGAGED_PSI ) ) ||
           ( pxData->rear_valid && ( pxData->rear.rear_brake > BRAKE_ENGAGED_PSI ) );
}

static bool bAppsBrakePlausibility( const TelemetrySnapshot_t *pxData, bool bFault )
{
    uint16_t usPedal = ( pxData->sensors.tps1 > pxData->sensors.tps2 ) ? pxData->sensors.tps1
                                                                         : pxData->sensors.tps2;
    bool bBrake = bBrakeEngaged( pxData );

    if( !pxData->sensors_valid )
    {
        return bFault;
    }

    if( bBrake && ( usPedal > APPS_BRAKE_TRIP_PCT ) )
    {
        return true;
    }

    if( usPedal < APPS_BRAKE_RESET_PCT )
    {
        return false;
    }

    return bFault;
}

static bool bFrameTimedOut( bool bValid, TickType_t xRxTick, TickType_t xNow )
{
    return !bValid || ( ( xNow - xRxTick ) > pdMS_TO_TICKS( SENSOR_CAN_TIMEOUT_MS ) );
}

/* SENSOR_TIMEOUT_* bits for the sensor frames that are missing right now.
 * Call with xTelemetryMutex held. */
static uint8_t u8CheckSensorTimeouts( const TelemetrySnapshot_t *pxData, TickType_t xNow )
{
    uint8_t u8Timeouts = 0U;

    if( bFrameTimedOut( pxData->sensors_valid, pxData->sensors_rx_tick, xNow ) )
    {
        u8Timeouts |= SENSOR_TIMEOUT_TPS;
    }
    if( bFrameTimedOut( pxData->front_axle_valid, pxData->front_axle_rx_tick, xNow ) )
    {
        u8Timeouts |= SENSOR_TIMEOUT_FRONT;
    }
    if( bFrameTimedOut( pxData->rear_valid, pxData->rear_rx_tick, xNow ) )
    {
        u8Timeouts |= SENSOR_TIMEOUT_REAR;
    }

    return u8Timeouts;
}

static void vSendMainEcuErrors( bool bAppsBrakeFault, uint8_t u8SensorTimeouts )
{
    uint8 data[ 8 ] = { 0U };

    data[ 0 ] = bAppsBrakeFault ? 1U : 0U;
    data[ 1 ] = u8SensorTimeouts;
    ( void ) canTransmit( canREG1, MAIN_ECU_ERRORS_MAILBOX, data );
}

/* Loop timing for the telemetry slow frame: wall time of one drivers loop
 * iteration (work only, without the vTaskDelay; preemption by CAN RX
 * counts). Longer than DRIVERS_PERIOD_MS is an overrun -- expect one each
 * time drive is enabled, since run_drivers() blocks ~15 ms in the
 * CiA 402 enable sequence. */
static void vRecordLoopTime( uint32_t ulCounts )
{
    uint32_t ulUs = ulCounts / ( RUN_TIME_COUNTER_HZ / 1000000U );

    xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
    if( ulUs > xTelemetryData.loop_time_max_us )
    {
        xTelemetryData.loop_time_max_us = ( ulUs > 0xFFFFU ) ? 0xFFFFU : ( uint16_t ) ulUs;
    }
    if( ( ulUs > ( DRIVERS_PERIOD_MS * 1000U ) ) && ( xTelemetryData.loop_overruns < 0xFFFFU ) )
    {
        xTelemetryData.loop_overruns++;
    }
    xSemaphoreGive( xTelemetryMutex );
}

static portTASK_FUNCTION( vDriversTask, pvParameters )
{
    bool bDriveEnabled = false;
    bool bAppsBrakeFault = false;
    bool bAppsBrakeFaultSent = false;
    uint8_t u8SensorTimeouts = 0U;
    uint8_t u8SensorTimeoutsSent = 0U;
    TickType_t xLastErrorsTx = 0U;
    int16_t s16Torque[ 2 ] = { 0, 0 };
    uint16_t usControlWord;
    int16_t s16TargetTorque;
    TickType_t xTxTick;
    uint32_t n;
    uint32_t ulLoopStart;
    ( void ) pvParameters;

    for( ;; )
    {
        ulLoopStart = portGET_RUN_TIME_COUNTER_VALUE();

        xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );

        /* Sensor frames missing -> treated like an implausibility: drive
         * forced off, needs a new cmd 7 once they're back. */
        u8SensorTimeouts = u8CheckSensorTimeouts( &xTelemetryData, xTaskGetTickCount() );
        xTelemetryData.sensor_timeouts = u8SensorTimeouts;
        if( u8SensorTimeouts != 0U )
        {
            xTelemetryData.drive_enabled = false;
        }

        /* Drive on/off from 0x600 (cmd 7/8), or forced off by 0x401 or a
         * sensor CAN timeout. */
        bDriveEnabled = xTelemetryData.drive_enabled;

        /* The average between APPS1 (tps1) and APPS2 (tps2), scaled to
         * 0-1000, goes through the per-motor torque map of the current
         * mode: normal (linear, capped at the DC current limit) or race. */
        xTelemetryData.race_map_limiting = false;
        if( xTelemetryData.sensors_valid )
        {
            if( xTelemetryData.race_mode )
            {
                float fAvgTps = ( float ) ( ( xTelemetryData.sensors.tps1 + xTelemetryData.sensors.tps2 ) * TPS_TO_TORQUE ) / ( 2.0f );
                s16Torque[ 0 ] = s16RaceTorqueMap( fAvgTps, &xTelemetryData.driver[ 0 ] );
                s16Torque[ 1 ] = s16RaceTorqueMap( fAvgTps, &xTelemetryData.driver[ 1 ] );
                /* Below the pedal -> the 1/velocity (DC current) fall-off kicked in. */
                xTelemetryData.race_map_limiting = ( ( float ) s16Torque[ 0 ] < fAvgTps ) ||
                                                   ( ( float ) s16Torque[ 1 ] < fAvgTps );
            }
            else
            {
                uint32_t ulAvgTps = ( ( ( uint32_t ) xTelemetryData.sensors.tps1 + ( uint32_t ) xTelemetryData.sensors.tps2 ) * TPS_TO_TORQUE ) / 2U;
                s16Torque[ 0 ] = s16NormalTorque( ulAvgTps, &xTelemetryData.driver[ 0 ] );
                s16Torque[ 1 ] = s16NormalTorque( ulAvgTps, &xTelemetryData.driver[ 1 ] );
            }
        }
        else
        {
            s16Torque[ 0 ] = 0;
            s16Torque[ 1 ] = 0;
        }

        xTelemetryData.torque_request[ 0 ] = s16Torque[ 0 ];
        xTelemetryData.torque_request[ 1 ] = s16Torque[ 1 ];

        /* Traction control (cmd 9 on / 0xA off): cuts each motor's
         * request while its rear wheel slips past the target. */
        if( xTelemetryData.traction_on && bDriveEnabled )
        {
            traction_apply( &xTelemetryData, s16Torque, ( float ) DRIVERS_PERIOD_MS / 1000.0f );
        }
        else
        {
            traction_reset();
        }

        xTelemetryData.torque_cmd[ 0 ] = s16Torque[ 0 ];
        xTelemetryData.torque_cmd[ 1 ] = s16Torque[ 1 ];
        xTelemetryData.tc_cutting[ 0 ] = ( s16Torque[ 0 ] < xTelemetryData.torque_request[ 0 ] );
        xTelemetryData.tc_cutting[ 1 ] = ( s16Torque[ 1 ] < xTelemetryData.torque_request[ 1 ] );

        /* EV.4.7: brake + pedal > 25 % forces drive off, exactly like a
         * 0x401 fault or cmd 8: the drives stop and stay stopped until the
         * normal start (brake + button -> cmd 7). They used to restart on
         * their own once the pedal was back under 5 %, which re-enabled the
         * inverters with the car still rolling and tripped both with
         * gate-driver faults (0xFF06 / 0xFF0A). cmd 7 is only accepted
         * once the pedal is under 5 % (the fault stays latched until then). */
        xTelemetryData.brake_engaged = bBrakeEngaged( &xTelemetryData );
        bAppsBrakeFault = bAppsBrakePlausibility( &xTelemetryData, bAppsBrakeFault );
        xTelemetryData.apps_brake_fault = bAppsBrakeFault;
        if( bAppsBrakeFault )
        {
            xTelemetryData.drive_enabled = false;
            bDriveEnabled = false;
            s16Torque[ 0 ] = 0;
            s16Torque[ 1 ] = 0;
            traction_reset();
            xTelemetryData.torque_cmd[ 0 ] = 0;
            xTelemetryData.torque_cmd[ 1 ] = 0;
            xTelemetryData.tc_cutting[ 0 ] = false;
            xTelemetryData.tc_cutting[ 1 ] = false;
        }

        xSemaphoreGive( xTelemetryMutex );

        run_drivers( bDriveEnabled, s16Torque );

        /* What actually went out on RXPDO1, for telemetry. */
        for( n = 0U; n < 2U; n++ )
        {
            if( drivers_last_rxpdo1( n, &usControlWord, &s16TargetTorque, &xTxTick ) )
            {
                xSemaphoreTake( xTelemetryMutex, portMAX_DELAY );
                xTelemetryData.driver[ n ].control_word = usControlWord;
                xTelemetryData.driver[ n ].target_torque = s16TargetTorque;
                xTelemetryData.driver[ n ].rxpdo1_valid = true;
                xTelemetryData.driver[ n ].rxpdo1_rx_tick = xTxTick;
                xSemaphoreGive( xTelemetryMutex );
            }
        }

        /* Tell the on-board ECU so the driver sees it: on every change,
         * then periodically while active. */
        if( ( bAppsBrakeFault != bAppsBrakeFaultSent ) ||
            ( u8SensorTimeouts != u8SensorTimeoutsSent ) ||
            ( ( bAppsBrakeFault || ( u8SensorTimeouts != 0U ) ) &&
              ( ( xTaskGetTickCount() - xLastErrorsTx ) >= pdMS_TO_TICKS( MAIN_ECU_ERRORS_PERIOD_MS ) ) ) )
        {
            vSendMainEcuErrors( bAppsBrakeFault, u8SensorTimeouts );
            bAppsBrakeFaultSent = bAppsBrakeFault;
            u8SensorTimeoutsSent = u8SensorTimeouts;
            xLastErrorsTx = xTaskGetTickCount();
        }

        vRecordLoopTime( portGET_RUN_TIME_COUNTER_VALUE() - ulLoopStart );

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
    gioInit();
    canInit();
    sciInit();
    sciSetBaudrate( sciREG, 115200U );

    /* Clears xTelemetryData, so the defaults below go after it. */
    telemetry_init( PRIO_TELEMETRY );
    xTelemetryData.drive_enabled = false; /* until cmd 7 on 0x600 */
    xTelemetryData.race_mode = false;     /* normal (linear) torque until cmd 0xC */
    xTelemetryData.traction_on = true;    /* traction control on until cmd 0xA */

    init_drivers();
    charger_init( PRIO_CHARGER );

    xTaskCreateStatic( vCanRxTask, "CanRx", STACK_CAN_RX, NULL,
                        PRIO_CAN_RX, xCanRxStack, &xCanRxTaskBuffer );

    xTaskCreateStatic( vDriversTask, "Drivers", STACK_DRIVERS, NULL,
                        PRIO_DRIVERS, xDriversStack, &xDriversTaskBuffer );

    vTaskStartScheduler();

    /* Only reached if there wasn't enough memory for the idle/timer task. */
    for( ;; )
    {
    }
}
