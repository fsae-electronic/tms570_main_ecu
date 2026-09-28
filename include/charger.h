#ifndef CHARGER_H_
#define CHARGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

/* ---- Charge setpoint ------------------------------------------------- */

#define CHARGER_CELL_COUNT          29
#define CHARGER_CELL_MAX_VOLTAGE    3.65f   /* LiFePO4 full-charge cell voltage */

/* In volts and amps, like VOLTAGE / CURRENT on the LPC board; the x10 is
 * done in charger_send(). */
#define CHARGER_VOLTAGE             (CHARGER_CELL_MAX_VOLTAGE * CHARGER_CELL_COUNT) /* 105.85 V */
#define CHARGER_CURRENT             (15.0f)

/* ---- Charger CAN (CAN2, 250 kbit/s, 29-bit IDs) ------------------------
 *
 * TX 0x1806E5F4 (ECU -> charger), every CHARGER_TX_PERIOD_MS. Same frame
 * as send_can2charger() on the LPC board:
 *   B0   voltage x10, high byte
 *   B1   voltage x10, low byte
 *   B2   0x10 (fixed)
 *   B3   current x10, low byte
 *   B4   0x00
 *   B5   rolling counter, +1 every frame (wraps 0xFF -> 0x00)
 *   B6-7 0x00
 * RX 0x18FFF4E5 (charger -> ECU):
 *   B0-1 output voltage x10 (big-endian)
 *   B2-3 output current x10 (big-endian)
 *   B4   status flags
 */
#define CHARGER_CAN_BAUDRATE        250000U
#define CHARGER_CAN_ID_TX           0x1806E5F4U
#define CHARGER_CAN_ID_RX           0x18FFF4E5U

/* ---- Reports to the on-board ECU (CAN1, 11-bit IDs, little-endian) -----
 *
 * "Charging" = a charger frame arrived within CHARGER_RX_TIMEOUT_MS.
 * CHARGER_CAN1_ID_CMD, sent every CHARGER_REPORT_PERIOD_MS while charging
 * and once with B0 = 0 when charging stops:
 *   B0 1 = charging, 0 = not charging
 *   B1 charger status flags (RX B4)
 * CHARGER_CAN1_ID_DATA, sent every CHARGER_REPORT_PERIOD_MS while charging:
 *   B0-1 commanded voltage (0.1 V/bit)
 *   B2-3 commanded current (0.1 A/bit)
 *   B4-5 measured voltage  (0.1 V/bit)
 *   B6-7 measured current  (0.1 A/bit)
 */
#define CHARGER_CAN1_ID_CMD         0x610U
#define CHARGER_CAN1_ID_DATA        0x611U

/* ---- Timing ---------------------------------------------------------- */

#define CHARGER_TASK_PERIOD_MS      50U
#define CHARGER_TX_PERIOD_MS        950U
#define CHARGER_REPORT_PERIOD_MS    500U
#define CHARGER_RX_TIMEOUT_MS       5000U

/**
 * @brief Initialize CAN2 for the charger, the CAN1 report mailboxes, and
 *        create the charger task. Call after canInit() and before
 *        vTaskStartScheduler().
 * @param priority FreeRTOS priority of the charger task.
 */
void charger_init(UBaseType_t priority);

#endif /* CHARGER_H_ */
