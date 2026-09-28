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

/* Driver PDO IDs are base + node, node 1 = left, node 2 = right. */
#define CAN_ID_RXPDO1_BASE   0x200U /* control word / target torque (commanded) */
#define CAN_ID_TXPDO1_BASE   0x180U /* status word / warnings / error */
#define CAN_ID_TXPDO2_BASE   0x280U /* motor velocity / rated current */
#define CAN_ID_TXPDO3_BASE   0x380U /* driver temp / DC bus voltage / motor temp */
#define CAN_ID_HEARTBEAT_BASE 0x700U /* CANopen NMT state */

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
     * (motor_velocity / 60), updated on every TXPDO2 -- not from 0x504,
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
    uint16_t control_word;
    int16_t  target_torque;
    bool     rxpdo1_valid;

    uint16_t status_word;
    uint16_t warnings;
    uint16_t error;
    bool     txpdo1_valid;

    int32_t  motor_velocity;
    uint32_t motor_rated_current;
    uint8_t  motor_temp;
    bool     txpdo2_valid;

    uint8_t  driver_temp;
    uint16_t driver_dc_voltage;
    bool     txpdo3_valid;

    uint8_t  heartbeat_state;
    bool     heartbeat_valid;
} DriverNode_t;

/* Latest-known values from every source, plus per-message "ever received"
 * flags so the telemetry task can tell real zeros from "never heard from
 * this source". Access only while holding xTelemetryMutex. */
typedef struct
{
    SensorsData_t   sensors;
    bool            sensors_valid;
    FrontAxleData_t front_axle;
    bool            front_axle_valid;
    CurrentData_t   current;
    bool            current_valid;
    RearData_t      rear;
    bool            rear_valid;
    uint8_t         calibration_type;
    uint8_t         calibration_value;
    bool            calibration_valid;
    bool            drive_enabled;   /* last drive cmd on 0x600: 7 on, 8 off (0x401 fault forces off) */
    bool            race_mode;       /* last mode cmd on 0x600: 0xB normal, 0xC race */
    DriverNode_t    driver[ 2 ]; /* [0] = left/node1, [1] = right/node2 */
} TelemetrySnapshot_t;

extern TelemetrySnapshot_t xTelemetryData;
extern SemaphoreHandle_t xTelemetryMutex;

#endif /* TELEMETRY_H */
