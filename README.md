# TMS570 Main ECU

Firmware for the **Main ECU** of ITBA Competición's Formula SAE electric car
(FSAE 2027 rules). The board is built around a **Texas Instruments
TMS570LS1224** (ARM Cortex-R4F in lockstep, big-endian) and runs
**FreeRTOS**.

This is a clean rewrite of
[`tms570_freertos`](https://github.com/fsae-electronic/tms570_freertos) that
targets `arm-none-eabi-gcc` instead of TI's CGT compiler.

---

## Contents

1. [What the Main ECU is](#what-the-main-ecu-is)
2. [Features](#features)
3. [Hardware](#hardware)
4. [Firmware architecture](#firmware-architecture)
5. [CAN frames](#can-frames)
6. [Control logic](#control-logic)
7. [Torque maps](#torque-maps)
8. [Traction control](#traction-control)
9. [Charger control](#charger-control)
10. [Telemetry](#telemetry)
11. [FSAE 2027 rules coverage](#fsae-2027-rules-coverage)
12. [Configuration constants](#configuration-constants)
13. [Unused board I/O](#unused-board-io)
14. [Repository structure](#repository-structure)
15. [Building](#building)
16. [Flashing](#flashing-segger-j-link)
17. [Known limitations and TODO](#known-limitations-and-todo)

---

## What the Main ECU is

The Main ECU is the car's **vehicle control unit (VCU)**. It sits between
the driver's inputs and the two motor controllers:

```
 adc2can_front ──┐ APPS, front brake, front wheel speeds, steering
 adc2can_rear  ──┤ rear brake, AC/DC currents
 on-board ECU  ──┤ drive on/off, mode, TC levels (dashboard)
                 │
               CAN1 (1 Mbit/s)
                 │
           ┌─────┴─────┐            CAN2 (250 kbit/s)
           │ MAIN ECU  ├──────────────────────────── charger
           └─────┬─────┘
                 │ CANopen (CiA 402)        UART 115200 ── XBee SX / USB
        ┌────────┴────────┐                               (telemetry)
   drive node 1      drive node 2
   (left motor)      (right motor)
```

It does not read any sensor directly. The pedal (APPS), brake pressure
(BSE), wheel speeds and steering are digitised by the **adc2can** boards and
arrive over CAN. The driver's commands come from the **on-board ECU**
(dashboard). The Main ECU checks that everything is plausible, turns the
pedal into a torque request per motor and sends it to the two drives over
CANopen.

The Main ECU can only **remove** torque. The Shutdown Circuit, BSPD, BOTS,
IMD and BMS are hardware and stay outside this board, as the rules require.

---

## Features

- **Two-motor torque control** over CANopen / CiA 402 (node 1 = left,
  node 2 = right), with a 50 ms torque setpoint.
- **Two driving modes:** *normal* (linear, capped at the DC current limit)
  and *race* (follows the pedal, then holds the DC current near the limit
  at high speed).
- **Traction control:** a PI loop per rear wheel on the slip ratio, tuned
  live from the dashboard with two levels (TC 0-10, TC Cut 1-10).
- **Safety checks:**
  - EV.4.7 APPS / brake plausibility.
  - APPS and brake implausibility reported by the adc2can boards.
  - 100 ms timeouts on the APPS and brake frames.
- **Fault reporting** to the dashboard on CAN ID 0x403.
- **Charger control** on a second CAN bus (CAN2), with status reports to the
  dashboard.
- **Telemetry** over UART (XBee SX 900 MHz radio, or USB) as framed and
  CRC-protected binary frames: a fast frame every 10 ms and a slow frame at
  1 Hz.

---

## Hardware

| Block | Parts |
|---|---|
| MCU | TI TMS570LS1224 (PGE 144), 16 MHz crystal |
| Power | 24 V GLV nominal (runs on 12 V, 36 V absolute max). 6 A fuse, reverse-polarity diode and TVS, then RT2872 buck (5 V), TPS78533 LDO (3.3 V) and TLV733 LDO (1.2 V core) |
| Supervision | TPS3106 supervisor (3.3 V / 1.2 V); `D_ERROR` LED on the MCU's nERROR pin |
| CAN | 2 × TJA1042. CAN1 (vehicle) and CAN2 (charger). Split 120 Ω termination on solder jumpers (J2+J3 for CAN1, J4+J5 for CAN2) |
| Telemetry | Digi XBee-SX PRO (900 MHz, 1 W), RP-SMA antenna (J6). CP2102N USB-UART bridge (micro-USB) |
| Debug | TI 20-pin JTAG (J_DEBUG); J_RST jumper resets the MCU |
| I/O (not used by the firmware yet) | 8 analog inputs (0-5 V), 8 digital inputs (5 V, opto-isolated), 2 frequency inputs (Hall / VR), 8 low-side outputs (NCV8405, 3 A fuse each) |
| Connector | J7, Cinch 581-01-60-005 (60 pins) in a Cinch 581-01-60-033 enclosure |

### J7 pinout

Looking at the ECU from the connector side, block L-Y is on the left, block
A-K is on the right, and row 1 is on top.

| | 1 | 2 | 3 | | | 1 | 2 | 3 |
|---|---|---|---|---|---|---|---|---|
| **L** | CAN1_L | **VIN** | GND | | **A** | DIN5 | DIN4 | GND |
| **M** | CAN1_H | FREQ2_IN | FREQ1_IN | | **B** | DIN0 | +5V | GND |
| **N** | +3V3 | +5V | GND | | **C** | DIN1 | +5V | GND |
| **P** | CAN2_L | +5V | GND | | **D** | DIN2 | +5V | GND |
| **R** | CAN2_H | +5V | GND | | **E** | DIN3 | DIN6 | DIN7 |
| **S** | AIN6 | AIN7 | GND | | **F** | DOUT6 | DOUT7 | GND |
| **T** | AIN3 | +5V | GND | | **G** | DOUT3 | +5V | GND |
| **W** | AIN2 | +5V | GND | | **H** | DOUT2 | +5V | GND |
| **X** | AIN1 | +5V | GND | | **J** | DOUT1 | +5V | GND |
| **Y** | AIN0 | AIN5 | AIN4 | | **K** | DOUT0 | DOUT5 | DOUT4 |

### LEDs

| LED | Meaning |
|---|---|
| D4 (green) | VIN present after fuse F1 |
| D2 (red) | VIN present before F1. If D2 is on and D4 is off, F1 is blown |
| D5 / D6 (green) | 5 V / 3.3 V rails OK |
| D_ERROR (red) | MCU error signalling module (ESM) flagged a fault. Needs a reset |
| TX/RX USB, CAN1, CAN2 (blue) | Bus activity |

---

## Firmware architecture

Four FreeRTOS tasks share one mutex-protected snapshot (`xTelemetryData`,
see `app/telemetry.h`):

| Task | Priority | Period | Role |
|---|---|---|---|
| `CanRx` (`app/main.c`) | 3 | 10 ms | Polls every CAN1 mailbox this ECU listens to and decodes each frame into the snapshot. It also handles the dashboard commands and the 0x401 faults. |
| `Drivers` (`app/main.c`) | 2 | 10 ms | Runs the safety checks, the torque map and traction control, then calls `run_drivers()` (`drivers/drivers.c`). Sends 0x403 and measures its own loop time. |
| `Telemetry` (`app/telemetry.c`) | 1 | 10 ms | Takes a consistent copy of the snapshot and sends the fast and slow frames over the UART. |
| `Charger` (`drivers/charger.c`) | 1 | 50 ms | Sends the charger setpoint on CAN2 and reports charging status on CAN1. |

CAN is **polled, not interrupt-driven**. `canGetData()` checks the hardware
NEWDAT flag and returns 0 when there is nothing new, so a short
`vTaskDelay` loop picks up new frames without any VIM routing. The CAN
interrupt line is left disabled in `canInit()` on purpose: the
HALCoGen-generated ISR relies on a TI CGT pragma that GCC doesn't support.

All tasks and the mutex are allocated **statically**
(`configSUPPORT_STATIC_ALLOCATION`).

---

## CAN frames

All multi-byte CAN fields are **little-endian** (B0 = LSB). This was checked
against the team's "Mensajes CAN" spec and measured on the car.

### CAN1, received

| ID | From | Mailbox | Content |
|---|---|---|---|
| `0x501` | adc2can_front | 1 | B0-1 APPS1 (TPS1), B2-3 APPS2 (TPS2), 0-100 % pedal travel. Every 10 ms |
| `0x502` | adc2can_front | 2 | B0-1 front-left wheel rpm, B2-3 front-right wheel rpm, B4-5 steering (raw), B6-7 front brake pressure (psi). Every 10 ms |
| `0x503` | adc2can_rear | 12 | B0-1 AC current 1, B2-3 AC current 2, B4-5 DC current 1, B6-7 DC current 2 (raw) |
| `0x504` | adc2can_rear | 13 | B0-1 rear brake pressure (psi). Every 10 ms |
| `0x401` | adc2can boards | 27 | B0 APPS implausibility, B1 brake implausibility. Sent once per fault |
| `0x600` | on-board ECU | 24 | B0 command id, B1 value (see [Commands](#commands-0x600)) |
| `0x6A0` | on-board ECU | 29 | B0 TC level 0-10, B1 TC Cut level 1-10. A byte out of range (e.g. `0xFF`) keeps the current level |
| `0x181` / `0x182` | drives (TXPDO1) | 14 / 15 | Status word, warnings, error. Every 500 ms |
| `0x281` / `0x282` | drives (TXPDO2) | 16 / 17 | B0-3 velocity actual value 0x606C (int32, motor rpm), B4-5 DC-link voltage 0x6079 (0.1 V). Every 10 ms |
| `0x381` / `0x382` | drives (TXPDO3) | 18 / 19 | B0-3 motor rated current 0x6075 (mA), B4 controller temperature, B5 motor temperature. Every 100 ms |
| `0x701` / `0x702` | drives (heartbeat) | 22 / 23 | CANopen NMT state |

### CAN1, transmitted

| ID | To | Content |
|---|---|---|
| `0x000` | drives | NMT: `0x01` operational, `0x80` pre-operational |
| `0x201` / `0x202` | drives (RXPDO1) | B0-1 control word, B2-3 target torque (per mille of rated torque, int16). The wire sign is inverted (`-torque`) to match how the motors are mounted |
| `0x403` | on-board ECU | B0 = 1 while EV.4.7 is active. B1 = sensor timeout bits: bit 0 `0x501`, bit 1 `0x502`, bit 2 `0x504`. Sent on every change, then every 100 ms while any fault is active |
| `0x610` | on-board ECU | B0 1 = charging / 0 = not charging, B1 charger status flags. Every 500 ms while charging, plus once when charging stops |
| `0x611` | on-board ECU | B0-1 commanded voltage, B2-3 commanded current, B4-5 measured voltage, B6-7 measured current (all 0.1 V or 0.1 A per bit). Every 500 ms while charging |

### CAN2 (charger, 250 kbit/s, 29-bit IDs)

| ID | Direction | Content |
|---|---|---|
| `0x1806E5F4` | ECU → charger | B0-1 voltage × 10 (big-endian), B2-3 current, B4 `0x00` (start), B5 rolling counter. Every 950 ms |
| `0x18FFF4E5` | charger → ECU | B0-1 output voltage × 10, B2-3 output current × 10 (big-endian), B4 status flags |

### Commands (0x600)

Byte 0 of `0x600`, the same values as the on-board ECU's
`calibration_cmd_id_t`:

| B0 | Action |
|---|---|
| `0x01`-`0x06` | Sensor calibrations for the adc2can boards (ignored here) |
| `0x07` | **Drive ON.** Clears a latched 0x401 fault and starts the drive enable sequence |
| `0x08` | **Drive OFF.** Drives go back to pre-operational |
| `0x09` / `0x0A` | Traction control ON / OFF |
| `0x0B` / `0x0C` | Normal mode / race mode |
| `0x0D` / `0x0E` | Telemetry stream ON / OFF |

---

## Control logic

### Boot state

At boot the ECU starts with:

- drive **disabled** (drives in pre-operational)
- **normal** mode
- traction control **on**, with TC level 5 and TC Cut level 5
- telemetry **on**

It also sends `TMS570 MAIN ECU (rewrite) INITIALIZED` over the UART.

### Drive enable

```
                 cmd 0x07 (on-board ECU: brake pressed + start button)
   ┌──────────┐ ─────────────────────────────────────────▶ ┌───────────┐
   │  STOP    │                                            │  DRIVE    │
   │ pre-op,  │ ◀───────────────────────────────────────── │ torque    │
   │ no torque│  cmd 0x08, 0x401 fault, sensor timeout,    │ every     │
   └──────────┘  EV.4.7 (brake + pedal > 25 %)              │ 50 ms     │
                                                           └───────────┘
```

When drive is enabled, `run_drivers()` runs the start-up sequence once per
drive, with 5 ms between frames:

1. NMT **operational**
2. Control word `0x0006` (Shutdown)
3. Control word `0x0007` (Switch On)
4. Control word `0x000F` (Enable Operation)

After that it sends `0x000F` + target torque on RXPDO1 every **50 ms**.
When drive is disabled, both nodes get NMT **pre-operational**.

Every fault forces drive **off**. Getting back needs a new drive-on
(`0x07`), which the dashboard only sends with the brake pressed and the
start button pushed.

### Safety checks (every 10 ms)

1. **Sensor timeouts (T.4.2.11 / T.4.3.5).** If `0x501` (APPS), `0x502`
   (front brake) or `0x504` (rear brake) hasn't arrived for more than
   **100 ms**, or never arrived, drive is forced off. The missing frames
   are reported in `0x403` B1. A corrupted frame is dropped by the CAN CRC
   and ends up as a timeout.
2. **APPS / brake implausibility (T.4.2.5, T.4.3.3).** The adc2can boards
   check the 10 % APPS deviation and the sensor ranges, and send `0x401`.
   Any non-zero byte forces drive off and latches `sensor_fault` until the
   next `0x07`.
3. **EV.4.7 APPS / brake plausibility.** If the brake is engaged (front
   *or* rear pressure > **50 psi**) and the **higher** of the two APPS reads
   more than **25 %**, torque goes to 0 and drive is forced off. The fault
   stays latched until both APPS read less than **5 %**. Only then is a
   new `0x07` accepted.

   The drives used to restart on their own once the pedal dropped under
   5 %. That re-enabled the inverters with the car still rolling and
   tripped both with gate-driver faults (`0xFF06` / `0xFF0A`), so restart
   now always goes through the normal start sequence.

---

## Torque maps

The pedal value is the **average of APPS1 and APPS2** (0-100 %) times
`TPS_TO_TORQUE` (10). That gives a 0-1000 request, in per mille of the
motor's rated torque. 1000 = the motor's rated current, which each drive
reports in TXPDO3 (`motor_rated_current`, mA).

The two motors run the same map independently, each with its own speed and
DC-link voltage. The mode is selected with `0x600` = `0x0B` / `0x0C`.
`mapas.png` plots both maps, and `tools/torque_map_3d.py` /
`tools/torque_sim.py` reproduce them.

### Normal mode (`s16NormalTorque`)

Linear with the pedal, scaled so that full pedal asks for the torque whose
phase current equals the battery DC limit:

```
torque_max = 1000 · MAX_DC_CURRENT / motor_rated_current     (capped at 1000)
torque     = pedal · torque_max / 1000
```

With `MAX_DC_CURRENT = 75 A` and a 250 A rated motor, full pedal = 300 ‰.
Until TXPDO3 arrives there is no current reference, so the request stays
at 0.

### Race mode (`s16RaceTorqueMap`)

The request follows the pedal up to a speed "knee". Above the knee it
falls off as 1/speed, so the DC current stays near `MAX_DC_CURRENT`:

```
v_max  = KV_EMRAX · V_dc                    (22 rpm/V × DC-link voltage)
M_min  = MAX_DC_CURRENT / I_rated           (e.g. 75 / 250 = 0.3)
M      = M_min · 1000 / pedal               (pedal ≤ 10 → M = 100, i.e. no limit)

if |v| > M · v_max:   torque = M_min · 1000 · v_max / |v|
else:                 torque = pedal
```

At full pedal the knee is `M_min · v_max` (about 0.3 · 2200 ≈ 660 rpm with
a ~100 V pack). At lower pedal positions it moves up proportionally. The
`TORQUE_LIMIT_POWER` bit in telemetry shows when the fall-off is active.
To get more torque at speed, raise `MAX_DC_CURRENT`: the knee scales
linearly with it. Keep it within the pack, BMS and fuse limits.

### Motor speed filter

Each TXPDO2 velocity sample is low-passed
(`VELOCITY_FILTER_ALPHA = 0.35` per 10 ms sample) before it is used by the
race map and traction control. The raw value is noisy enough to make both
jump from one cycle to the next. The filter is seeded with the first
sample.

Rear wheel speed = filtered motor speed / `GEAR_RATIO` (4).

---

## Traction control

`app/traction.c`: one PI loop per rear motor. It only ever **reduces** the
request (EV.3.1.3). The undriven front wheels give the ground speed:

```
slip   = (w_rear − w_front − TC_SLIP_SPEED_ALLOW) / w_rear
error  = slip − target_slip
integral += ki · error · dt         (ki = TC_KI_RELEASE when error < 0)
target = clamp(kp · error + integral, 0, max_cut)
cut    → target, at most rise_rate/s up and TC_CUT_FALL_RATE/s down
torque = torque · (1 − cut)
```

- `TC_SLIP_SPEED_ALLOW` (10 rpm) ignores small absolute differences at low
  speed, where a few rpm look like a huge ratio.
- TC stays out of the way below `TC_MIN_WHEEL_SPEED` (30 rpm of rear wheel
  speed), when `0x502` hasn't arrived, or at TC level 0. In those cases any
  cut still applied fades out instead of snapping back.
- The front speed is low-passed with `TC_FRONT_TAU_S` (30 ms).
- The rise/fall rate limits and `max_cut` stop the torque from stepping
  100 → 0 → 100 in one cycle, which the driver feels as a hit through the
  driveline.

Like a race car's TC / TC Cut knobs, the dashboard sets two levels on
`0x6A0`:

| TC level (B0) | 0 | 1 | 2 | 3 | 4 | **5** | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Target slip | off | 0.40 | 0.34 | 0.29 | 0.24 | **0.20** | 0.17 | 0.14 | 0.11 | 0.08 | 0.06 |

| TC Cut level (B1) | 1 | 2 | 3 | 4 | **5** | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| kp | 0.3 | 0.4 | 0.5 | 0.65 | **0.8** | 1.0 | 1.2 | 1.5 | 1.8 | 2.2 |
| ki (1/s) | 1.0 | 1.5 | 2.0 | 2.5 | **3.0** | 4.0 | 5.0 | 6.5 | 8.0 | 10.0 |
| max cut | 0.40 | 0.50 | 0.60 | 0.70 | **0.80** | 0.85 | 0.90 | 0.90 | 0.95 | 0.95 |
| rise rate (1/s) | 4 | 5 | 6.5 | 8 | **10** | 12 | 15 | 18 | 22 | 25 |

A higher TC level steps in earlier. A higher TC Cut level cuts harder and
faster. 5 / 5 is the default at boot.

> **Test workaround:** the front-right wheel sensor currently reads wrong,
> so front-left is used as the ground speed for both sides
> (`traction_apply()`). Once FR is fixed, filter each front on its own.

---

## Charger control

`drivers/charger.c`. CAN2 runs at 250 kbit/s with 29-bit IDs, and only
carries the charger.

- Every **950 ms** the ECU sends the setpoint frame `0x1806E5F4`:
  **105.85 V** (29 LiFePO4 cells × 3.65 V). With the current frame
  contents the charger delivers about **10 A**.
- The charger's reply `0x18FFF4E5` marks the ECU as *charging*. After
  **5 s** without a reply it counts as disconnected.
- While charging, the ECU reports `0x610` / `0x611` to the dashboard every
  500 ms. When charging stops it sends `0x610` once with B0 = 0.

Stopping the charge safely is done in hardware by the Charging Shutdown
Circuit (charger button, BMS, IMD; EV.9.3 / EV.9.4). The ECU is not part
of that path.

---

## Telemetry

`app/telemetry.c` sends **frame v3** over the SCI UART (pins 38/39,
115200 8N1). The UART goes to the XBee radio and, with the `J_EN_USB`
jumper fitted, also to the USB port. All fields are little-endian and are
serialized field by field (the CPU is big-endian, so a struct is never
`memcpy`'d).

| | Fast frame | Slow frame |
|---|---|---|
| Type | 1 | 2 |
| Size | 130 bytes | 60 bytes |
| Rate | every 10 ms | 1 Hz |
| Content | valid / fresh masks, ECU state, torque-limit reason, pedals, brakes, steering, 4 wheel speeds, torque request and per-motor command, currents, 2 × 23-byte drive blocks | firmware version + git hash, uptime, CPU load, loop overruns / worst loop time, active faults, CAN1 TEC/REC, motor rated currents |

Common header: `0xAA 0x55`, version (3), type, seq (per type), payload
length, `ecu_time_ms` (u32). It ends with a **CRC-16/CCITT-FALSE** over
version … end of payload. The full byte map is in the "Wire frame (v3)"
comment in `app/telemetry.c`. Fields with no source yet (BMS, IMU,
coolant, LV, TS state) are sent as 0.

Status bits in the fast frame:

| `ecu_state` bit | Meaning | `torque_limit_reason` bit | Meaning |
|---|---|---|---|
| 0 | Race mode | 0 | Drive disabled |
| 2 | Drive enabled | 1 | APPS fault (0x401 or 0x501 timeout) |
| 4 | TC enabled | 2 | EV.4.7 active |
| 5 / 6 | TC cutting left / right | 3 | Traction control cutting |
| 7 | APPS implausible | 4 | Race map DC-current fall-off |
| 8 | EV.4.7 active | 11 | Drive enabled but a drive is not in *Operation enabled* |
| 9 | Brake pressed | | |
| 10 | Safe mode (a fault forced torque to 0) | | |

The stream can be switched off with `0x600` = `0x0E` and back on with
`0x0D`. `tools/decode_telemetry.py` decodes it directly from the serial
port.

**XBee configuration mode:** set `XBEE_CONFIG_MODE` to 1 in
`app/telemetry.c` and flash. The SCI pins then stay high-Z and nothing is
sent, so the radio can be configured from XCTU. Set it back to 0 afterwards.

---

## FSAE 2027 rules coverage

| Rule | Requirement | How it is handled | Status |
|---|---|---|---|
| T.4.2.4-5 | APPS deviation > 10 % for > 100 ms → stop motor power | adc2can_front detects it and sends 0x401; Main ECU forces drive off | ✅ |
| T.4.2.9-11, T.4.3.5 | Sensor failure, lost or corrupted frames, timeouts | 100 ms timeout on 0x501 / 0x502 / 0x504; CAN CRC; 0x401 | ✅ |
| T.4.3.3 | BSE failure > 100 ms → stop motor power | 0x401 B1 + brake frame timeouts | ✅ |
| EV.4.7 | Brake + APPS > 25 % → cut until APPS < 5 % | Main ECU, latched; restart needs a new drive-on | ✅ |
| EV.3.1.1 | No reverse | Torque request is always ≥ 0 | ✅ |
| EV.3.1.3 | Algorithms may only reduce torque | TC multiplies by (1 − cut), cut ∈ [0, max_cut] | ✅ |
| EV.3.3.1 | 80 kW max | 75 A DC per drive (~16 kW at ~106 V) | ✅ |
| EV.3.3.3 | No regen below 5 km/h | No regeneration at all | ✅ |
| EV.10.6.2 | Ready to Drive only with TS active + brake + manual action | Brake + button: on-board ECU. **TS active: not checked** | ⚠️ |
| EV.10.7 | Ready to Drive sound (1-3 s, ≥ 80 dBA) | **Not implemented** | ❌ |

---

## Configuration constants

| Constant | File | Value | Meaning |
|---|---|---|---|
| `TPS_TO_TORQUE` | `app/main.c` | 10 | Pedal % → per mille of rated torque. 1 = bench testing (pedal reaches only 10 % of the range) |
| `MAX_DC_CURRENT` | `app/main.c` | 75 A | DC current limit per drive (normal-mode cap, race-mode knee) |
| `KV_EMRAX` | `app/main.c` | 22 rpm/V | Motor KV for the race map |
| `GEAR_RATIO` | `app/main.c` | 4 | Motor turns per rear wheel turn |
| `VELOCITY_FILTER_ALPHA` | `app/main.c` | 0.35 | Motor speed low-pass |
| `BRAKE_ENGAGED_PSI` | `app/main.c` | 50 psi | "Brake engaged" threshold (EV.4.7) |
| `APPS_BRAKE_TRIP_PCT` / `_RESET_PCT` | `app/main.c` | 25 % / 5 % | EV.4.7 thresholds |
| `SENSOR_CAN_TIMEOUT_MS` | `app/main.c` | 100 ms | APPS / brake frame timeout |
| `CAN_RX_PERIOD_MS`, `DRIVERS_PERIOD_MS` | `app/main.c` | 10 ms | Task periods |
| `TORQUE_SEND_PERIOD_MS` | `drivers/drivers.c` | 50 ms | RXPDO1 torque rate (keep in sync with `tools/torque_sim.py`) |
| `TC_*` | `app/traction.h` | see above | Traction control |
| `TELEMETRY_PERIOD_MS`, `XBEE_CONFIG_MODE` | `app/telemetry.c` | 10 ms, 0 | Telemetry |
| `CHARGER_CELL_COUNT`, `CHARGER_CELL_MAX_VOLTAGE` | `include/charger.h` | 29, 3.65 V | Charge voltage |

---

## Unused board I/O

The board has general-purpose I/O on J7 that the firmware doesn't use yet.
They are meant for things like a pump or fan relay, the brake light, or a
Ready to Drive buzzer.

| I/O | Electrical | MCU pins | How to use from code |
|---|---|---|---|
| **DI0-7** digital inputs | 5 V, active high, opto-isolated (~8 mA at 5 V). Don't connect 12/24 V | DI0-5: GIOA[0,1,2,5,6,7]; DI6-7: GIOB[0,1] | `gioGetBit(gioPORTA, bit)` / `gioGetBit(gioPORTB, bit)`. `gioInit()` already sets them as inputs |
| **DO0-7** digital outputs | Low-side: load between +GLV and DOx. NCV8405 (current limit, thermal shutdown, 42 V clamp for inductive loads), 3 A fast fuse per output; ≤ 2 A continuous recommended. Off during reset | DO0-1: GIOB[2,3]; DO2-7: N2HET1[0..5] | `gioSetDirection(gioPORTB, 0x0C)` + `gioSetBit(gioPORTB, 2, 1)`; `gioSetDirection(hetPORT1, 0x3F)` + `gioSetBit(hetPORT1, n, 1)`. **The current `pinmux.c` maps pins 23/24/25/30/31 to SPI4 and GIOB[2] to pin 55, so regenerate the pinmux in HALCoGen first** |
| **AI0-7** analog inputs | 0-5 V, 100 kΩ to GND (open input reads 0 V), clamped, buffered, 2/3 divider, ~24 Hz RC filter; ~1.2 mV per count at the pin | AD1IN[0..7] | Add HALCoGen `adc.c` (group 1 = channels 0-7): `adcStartConversion(adcREG1, adcGROUP1)` → `adcGetData(...)`. `V_in = raw · 3.3 / 4095 · 1.5` |
| **FREQ1-2** frequency inputs | Hall (open collector, pull-up on board) or VR sensor on a toothed wheel | FREQ1: N2HET1[15] (pin 41); FREQ2: N2HET1[10] (pin 118) | Add HALCoGen `het.c` with capture channels on those pins: `capGetSignal(hetRAM1, capN, &sig)` gives the period in µs. `rpm = 60e6 / (period_us · N_teeth)`. Use a timeout (no edge for ~200 ms → 0 rpm). At high speed, count edges over a fixed window instead (`edgeGetCounter()`) |

> Bench-check the FREQ input levels before relying on them. The on-board
> bias (10 k series, 10 k to 5 V, 20 k to GND) puts the MCU pin at about
> 2.0 V with the input at 0 V.

---

## Repository structure

```
app/          Application: main.c (CAN RX, safety logic, torque maps, drivers task),
              traction.c/.h (traction control), telemetry.c/.h (shared snapshot + UART frames)
drivers/      Peripheral drivers (CAN, SCI, GIO, RTI) -- HALCoGen-seeded, hand-fixed for GCC;
              canopen.c (NMT/SDO/PDO), drivers.c (CiA 402 drive state machine), charger.c (CAN2)
startup/      CPU/system init, VIM interrupt table, HALCoGen-generated device code
freertos/     FreeRTOS kernel + the Cortex-R4 port (port.c/portASM.s)
include/      All headers (TI register definitions, FreeRTOS, driver headers)
linker/       Linker script
port/         newlib syscall stubs for the freestanding build
tools/        decode_telemetry.py (telemetry decoder), torque_sim.py / torque_map_3d.py (torque map simulation)
mapas.png     Normal and race torque maps
```

---

## Building

You need the custom big-endian `arm-none-eabi-gcc` toolchain: stock distros
only ship little-endian runtime libraries. Build it from the toolchain
repo:

```bash
git clone git@github.com:fsae-electronic/tms570_debug_test.git
docker build -t tms570-toolchain tms570_debug_test/
```

Then build this firmware against it:

```bash
docker run --rm -v "$(pwd):/project" -w /project tms570-toolchain make
```

Or, with the toolchain installed natively at `/opt/arm-none-eabi-be`:

```bash
make
```

On **Windows**, with the project on Google Drive (`G:`), Docker Desktop
can't bind-mount the folder. Use `build.bat` instead. It streams the
sources into the container with `tar`, builds there, and copies the
outputs back. It also passes the git hash and dirty flag that end up in the
telemetry slow frame. Extra arguments go to `make`, e.g. `build.bat -j8`.

## Flashing (SEGGER J-Link)

Connect the J-Link to **J_DEBUG** (TI 20-pin JTAG):

```bash
JLinkExe -device TMS570LS1224 -if JTAG -speed 4000 -autoconnect 1 -jtagconf -1,-1 -CommanderScript flash.jlink
```

where `flash.jlink` is:

```
r
h
loadbin tms570_main_ecu.bin 0x00000000
r
g
qc
```

To reset the board by hand, short the **J_RST** jumper.

---

## Known limitations and TODO

- **Ready to Drive (EV.10.6.2):** nothing checks that the Tractive System
  is active before drive-on. It could be done in the on-board ECU, or here
  from the DC-link voltage the drives report in TXPDO2.
- **Ready to Drive sound (EV.10.7):** not implemented. One option is a
  buzzer on a DO output.
- **Front-right wheel speed:** the sensor reads wrong, so TC uses
  front-left for both sides (test workaround in `traction.c`).
- **Telemetry bandwidth:** a 130-byte fast frame takes about 11 ms at
  115200 baud, which is longer than its 10 ms period. The effective rate is
  limited by the UART.
- **Telemetry placeholders:** BMS, IMU, coolant, LV voltage, TS state, reset
  count/cause, drive torque/current actual and motor rated torque are sent
  as 0 until a source exists.
- **SAE compliance hook:** there is a placeholder in `vTelemetryTask`
  between reading the snapshot and sending it. It is not implemented, on
  purpose.
- **CAN is polled:** see [Firmware architecture](#firmware-architecture).
