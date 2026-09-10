# TMS570 Main ECU (rewrite)

Firmware for the Main ECU on a **Texas Instruments TMS570LS1224** (ARM
Cortex-R4, big-endian), running **FreeRTOS**. This is a clean rewrite of
[`tms570_freertos`](https://github.com/fsae-electronic/tms570_freertos),
targeting `arm-none-eabi-gcc` instead of TI's CGT compiler.

Pipeline: a CAN RX task polls every mailbox this ECU listens to (TPS,
front/rear axle sensors, both motor drivers' CANopen PDOs, calibration
commands) and updates a mutex-protected snapshot; a telemetry task wakes
periodically, takes a consistent copy, and sends it out over SCI/UART as
a fixed-size binary frame.

---

## Repository structure

```
app/          Application logic: main.c (tasks, CAN decode, SCI framing), telemetry.h
drivers/      Peripheral drivers (CAN, SCI, GIO, RTI) -- HALCoGen-seeded, hand-fixed for GCC
startup/      CPU/system init, VIM interrupt table, HALCoGen-generated device code
freertos/     FreeRTOS kernel + the Cortex-R4 port (port.c/portASM.s)
include/      All headers (TI register definitions, FreeRTOS, driver headers)
linker/       Linker script
port/         newlib syscall stubs for the freestanding build
tools/        decode_telemetry.py -- standalone host-side debug decoder
```

---

## Building

Requires the custom big-endian `arm-none-eabi-gcc` toolchain (stock
distros ship little-endian-only runtime libraries). Build it from the
toolchain repo:

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

`BENCH_TEST_SIMULATE_CAN=1 make` builds with a synthetic CAN-data
generator task instead of relying on real CAN hardware, for bench
testing the SCI pipeline alone.

## Flashing (SEGGER J-Link)

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

---

## Telemetry frame

`vPackFrame()` in `app/main.c` sends a 70-byte little-endian frame over
SCI at 115200 baud, every 100 ms:

```
offset  size  field
0       2     sync (0xAA 0x55)
2       1     version (2)
3       2     valid_mask (bit set per message received at least once)
5       2     tps1
7       2     tps2
9       2     wheel_speed_fl
11      2     wheel_speed_fr
13      2     steering_angle
15      2     front_brake
17      2     ac_current_1
19      2     ac_current_2
21      2     dc_current_1
23      2     dc_current_2
25      2     rear_brake
27      1     calibration_type
28      21    driver[0] (left):  control_word(2) target_torque(2)
              status_word(2) warnings(2) error(2) motor_velocity(2)
              motor_rated_current(4) motor_temp(1) driver_temp(1)
              driver_dc_voltage(2) heartbeat_state(1)
49      21    driver[1] (right), same layout as driver[0]
70      --    end
```

`tools/decode_telemetry.py` decodes and prints this stream directly off
`/dev/ttyUSB0`. To feed it into a live dashboard instead, see
[`telemetry-host`](https://github.com/fsae-electronic/telemetry-host)'s
`utils/tms570_bridge.py`, which re-encodes these frames into that
project's COBS/MessagePack protocol.

CAN message IDs, field layout, and byte order (little-endian) are
documented in `app/telemetry.h` and match the team's "Mensajes CAN" spec,
mirrored in `can-sniffer`'s `cansniff/messages.py`.

---

## Known limitations

- SAE compliance rule checks between reading the telemetry snapshot and
  sending it are not implemented yet (deliberately -- see the TODO at
  `vTelemetryTask` in `app/main.c`).
- CAN is polled (`canGetData()`), not interrupt-driven. CAN's own
  interrupt line is deliberately left disabled in `canInit()` -- see the
  comment there for why (a HALCoGen-generated ISR relies on a TI CGT
  pragma GCC doesn't support).
