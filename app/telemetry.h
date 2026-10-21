/*
 * Shared telemetry data model.
 *
 * Message layout/IDs match the team's CAN spec ("Mensajes CAN" doc /
 * can-logger's cansniff/messages.py catalog): all multi-byte fields on
 * the wire are little-endian (B0 = LSB). Byte order was previously
 * (mis)decoded as big-endian here -- confirmed wrong empirically (a
 * "direction" reading of ~47616 with the sensor at its expected ~180
 * position turns into 186 once byte-swapped) as well as against the
 * written spec, which says "B0 (LSB), B1 (MSB)" for 0x501/0x502
 * explicitly and follows CANopen's own (always little-endian) PDO
 * convention for the driver messages.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "os_semphr.h"

#define CAN_ID_TPS_DATA      0x501U /* adc2can_front: TPS1/TPS2 */
#define CAN_ID_FRONT_DATA    0x502U /* adc2can_front: front wheel speed/steering/brake */
#define CAN_ID_CURRENT_DATA  0x503U /* adc2can_rear: driver AC/DC currents */
#define CAN_ID_REAR_DATA     0x504U /* adc2can_rear: rear brake pressure */
#define CAN_ID_CALIBRATION   0x600U /* on-board ECU: calibration command */
#define CAN_ID_SENSOR_ERRORS 0x401U /* adc2can_front/rear, sent once per fault: B0 TPS implausibility, B1 brake implausibility */
#define CAN_ID_MAIN_ECU_ERRORS 0x403U /* main ECU -> on-board ECU: B0 EV.4.7 APPS/brake plausibility active */

/* Driver PDO IDs are base + node, node 1 = left, node 2 = right. */
#define CAN_ID_RXPDO1_BASE   0x200U /* control word / target torque (commanded) */
#define CAN_ID_TXPDO1_BASE   0x180U /* status word / warnings / error (500 ms) */
#define CAN_ID_TXPDO2_BASE   0x280U /* motor velocity / DC link voltage (10 ms) */
#define CAN_ID_TXPDO3_BASE   0x380U /* motor rated current / controller temp / motor temp (100 ms) */
#define CAN_ID_HEARTBEAT_BASE 0x700U /* CANopen NMT state */

/* TelemetrySnapshot_t.sensor_timeouts bits (set by main.c's drivers task):
 * sensor frame missing for more than SENSOR_CAN_TIMEOUT_MS. */
#define SENSOR_TIMEOUT_TPS      ( 1U << 0 ) /* 0x501 */
#define SENSOR_TIMEOUT_FRONT    ( 1U << 1 ) /* 0x502 */
#define SENSOR_TIMEOUT_REAR     ( 1U << 2 ) /* 0x504 */

typedef struct
{
    uint16_t tps1;
    uint16_t tps2;
} SensorsData_t;

typedef struct
{
    uint16_t wheel_speed_fl;
    uint16_t wheel_speed_fr;
    uint16_t steering_angle;
    uint16_t front_brake;
} FrontAxleData_t;

typedef struct
{
    uint16_t ac_current_1;
    uint16_t ac_current_2;
    uint16_t dc_current_1;
    uint16_t dc_current_2;
} CurrentData_t;

typedef struct
{
    uint16_t rear_brake;     /* 0x504 */
    /* Rear wheel speeds, derived from each motor's TXPDO2 velocity
     * (motor_velocity_f / GEAR_RATIO, see main.c), updated on every TXPDO2 -- not from 0x504,
     * so rear_valid doesn't cover them (see driver[n].txpdo2_valid).
     * Not sent over the telemetry frame. */
    int32_t  wheel_speed_rl; /* driver[0], node 1 = left */
    int32_t  wheel_speed_rr; /* driver[1], node 2 = right */
} RearData_t;

/* One motor driver node's worth of CANopen PDO data (0x201/0x181/0x281/0x381/0x701
 * for node 1, +1 each for node 2). Each sub-message has its own valid flag since
 * they arrive independently and at different rates. */
typedef struct
{
    /* RXPDO1 is what this ECU commands (the CAN controller never receives
     * its own frames), so it's filled from drivers.c's last send. */
    uint16_t control_word;
    int16_t  target_torque;       /* same sign as torque_cmd (+ = forward) */
    bool     rxpdo1_valid;
    TickType_t rxpdo1_rx_tick;    /* last RXPDO1 sent */

    uint16_t status_word;
    uint16_t warnings;
    uint16_t error;
    bool     txpdo1_valid;
    TickType_t txpdo1_rx_tick;

    int32_t  motor_velocity;      /* 0x606C velocity actual value */
    float    motor_velocity_f;    /* motor_velocity low-passed (VELOCITY_FILTER_ALPHA, main.c) */
    uint16_t driver_dc_voltage;   /* 0x6079 DC link circuit voltage (0.1 V/bit) */
    bool     txpdo2_valid;
    TickType_t txpdo2_rx_tick;

    uint32_t motor_rated_current; /* 0x6075 motor rated current (mA) */
    uint8_t  driver_temp;         /* 0x2026 controller temperature */
    uint8_t  motor_temp;          /* 0x2025 motor temperature */
    bool     txpdo3_valid;
    TickType_t txpdo3_rx_tick;

    uint8_t  heartbeat_state;
    bool     heartbeat_valid;
    TickType_t heartbeat_rx_tick;
} DriverNode_t;

/* Latest-known values from every source, plus per-message "ever received"
 * flags so the telemetry task can tell real zeros from "never heard from
 * this source". Access only while holding xTelemetryMutex. */
typedef struct
{
    SensorsData_t   sensors;
    bool            sensors_valid;
    TickType_t      sensors_rx_tick;    /* last 0x501, for the CAN timeout */
    FrontAxleData_t front_axle;
    bool            front_axle_valid;
    TickType_t      front_axle_rx_tick; /* last 0x502 */
    CurrentData_t   current;
    bool            current_valid;
    TickType_t      current_rx_tick;    /* last 0x503 */
    RearData_t      rear;
    bool            rear_valid;
    TickType_t      rear_rx_tick;       /* last 0x504 */
    uint8_t         calibration_type;
    uint8_t         calibration_value;
    bool            calibration_valid;
    TickType_t      calibration_rx_tick; /* last 0x600 */
    bool            drive_enabled;   /* last drive cmd on 0x600: 7 on, 8 off (0x401 fault forces off) */
    bool            race_mode;       /* last mode cmd on 0x600: 0xB normal, 0xC race */
    bool            traction_on;     /* last traction cmd on 0x600: 9 on, 0xA off (starts on) */
    int16_t         torque_request[ 2 ]; /* mode map output per motor, before TC, 0-1000 */
    int16_t         torque_cmd[ 2 ]; /* torque request per motor after mode map + TC, 0-1000 ([0] left, [1] right) */
    bool            tc_cutting[ 2 ];  /* TC cut this motor's request in the last cycle */
    bool            race_map_limiting; /* race map's 1/velocity fall-off active on either motor */
    bool            brake_engaged;    /* front or rear pressure above BRAKE_ENGAGED_PSI */
    bool            apps_brake_fault; /* EV.4.7 APPS / brake plausibility: motors stopped */
    bool            sensor_fault;     /* 0x401 TPS/brake implausibility, latched until the next drive-on */
    uint8_t         sensor_timeouts;  /* SENSOR_TIMEOUT_* bits: sensor frames missing */
    uint16_t        loop_overruns;    /* drivers loop iterations longer than its period, since boot (saturates) */
    uint16_t        loop_time_max_us; /* worst drivers loop iteration; cleared by each slow frame */
    DriverNode_t    driver[ 2 ]; /* [0] = left/node1, [1] = right/node2 */
} TelemetrySnapshot_t;

extern TelemetrySnapshot_t xTelemetryData;
extern SemaphoreHandle_t xTelemetryMutex;

/**
 * @brief Clear xTelemetryData, create xTelemetryMutex and the telemetry
 *        task, and send the boot message (or put the SCI pins in high-Z
 *        in XBee config mode). Call once before vTaskStartScheduler(),
 *        after sciInit().
 * @param uxPriority Telemetry task priority.
 */
void telemetry_init( UBaseType_t uxPriority );

/**
 * @brief Turn the UART stream on or off (cmd 0xD / 0xE on 0x600). Call
 *        with xTelemetryMutex held.
 */
void telemetry_set_enabled( bool bEnabled );

#endif /* TELEMETRY_H */
