#!/usr/bin/env python3
"""
Decode the Main ECU's telemetry stream on /dev/ttyUSB0.

Frame format (little-endian, 70 bytes) -- keep this in lockstep with
vPackFrame() in app/main.c:

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
    28      21    driver[0] (left):  control_word(H) target_torque(h)
                  status_word(H) warnings(H) error(H) motor_velocity(H)
                  motor_rated_current(I) motor_temp(B) driver_temp(B)
                  driver_dc_voltage(H) heartbeat_state(B)
    49      21    driver[1] (right), same layout as driver[0]
    70      --    end

Message catalogue / CAN IDs: see can-logger/cansniff/messages.py and the
team's "Mensajes CAN" spec doc. Every multi-byte CAN payload field is
little-endian (confirmed against telemetry.h's comment on how that was
verified).

No external dependencies -- configures the serial port directly via the
standard-library termios module instead of requiring pyserial.

Usage:
    python3 decode_telemetry.py [/dev/ttyUSB0] [baud]
"""
import sys
import os
import struct
import termios

FRAME_SIZE = 70
_NODE_FMT = "HhHHHHIBBHB"  # 21 bytes: see driver[] layout above
STRUCT_FMT = "<BBBH" + "HH" + "HHHH" + "HHHH" + "H" + "B" + (_NODE_FMT * 2)
assert struct.calcsize(STRUCT_FMT) == FRAME_SIZE

VALID_TPS, VALID_FRONT, VALID_CURRENT, VALID_REAR, VALID_CAL = (1 << i for i in range(5))


def valid_rxpdo1(n):    return 1 << (5 + 5 * n)
def valid_txpdo1(n):    return 1 << (6 + 5 * n)
def valid_txpdo2(n):    return 1 << (7 + 5 * n)
def valid_txpdo3(n):    return 1 << (8 + 5 * n)
def valid_heartbeat(n): return 1 << (9 + 5 * n)


NMT_STATES = {0x00: "Bootup", 0x05: "Operational", 0x7F: "Pre-operational", 0x04: "Stopped"}

STATUSWORD_BITS = {
    0: "RdyToSwitchOn", 1: "SwitchedOn", 2: "OpEnabled", 3: "Fault",
    4: "VoltageEnabled", 5: "QuickStop", 6: "SwitchOnDisabled", 7: "Warning",
    8: "MfrSpecific", 9: "Remote", 10: "TargetReached", 11: "InternalLimit",
}
WARNING_BITS = {
    0: "CtrlTempExceeded", 1: "MotorTempExceeded", 2: "DCLinkUndervoltage",
    3: "DCLinkOvervoltage", 4: "DCLinkOvercurrent", 5: "StallProtection",
    6: "MaxVelocityExceeded", 7: "BMSProposedPower", 8: "CapTempExceeded",
    9: "I2T_Protection", 10: "FieldWeakening",
}
DRIVER_ERRORS = {
    0x0000: "No fault", 0xFF01: "CURRENT_A", 0xFF02: "CURRENT_B",
    0xFF03: "HS_FET", 0xFF04: "LS_FET", 0xFF05: "DRV_LS_L1",
    0xFF06: "DRV_LS_L2", 0xFF07: "DRV_LS_L3", 0xFF08: "DRV_HS_L1",
    0xFF09: "DRV_HS_L2", 0xFF0A: "DRV_HS_L3", 0xFF0B: "MOTOR_FEEDBACK",
    0xFF0C: "DC_LINK_UNDERVOLTAGE", 0xFF0D: "PULS_MODE_FINISHED",
    0xFF0E: "APP_ERROR", 0xFF0F: "STO_ERROR",
    0xFF10: "CONTROLLER_OVERTEMPERATURE", 0x3210: "DC_LINK_OVERVOLTAGE",
}
CALIBRATION_TYPES = {
    0: "-", 1: "TPS 0%", 2: "TPS 100%", 3: "Steering left",
    4: "Steering centre", 5: "Steering right", 6: "Current sensors",
}


def bits(value, table):
    names = [name for bit, name in sorted(table.items()) if value >> bit & 1]
    return "|".join(names) if names else "-"


def open_serial(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    baud_const = getattr(termios, f"B{baud}")
    attrs[4] = baud_const  # ispeed
    attrs[5] = baud_const  # ospeed
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, no flow control
    attrs[3] = 0  # lflag: raw, no echo
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_exact(fd, n):
    buf = b""
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            raise EOFError("serial port closed")
        buf += chunk
    return buf


def find_sync(fd):
    prev = None
    while True:
        b = read_exact(fd, 1)[0]
        if prev == 0xAA and b == 0x55:
            return
        prev = b


def render_node(n, side, d, mask):
    (control_word, target_torque, status_word, warnings, error,
     motor_velocity, motor_rated_current, motor_temp, driver_temp,
     driver_dc_voltage, heartbeat_state) = d

    any_valid = mask & (valid_rxpdo1(n) | valid_txpdo1(n) | valid_txpdo2(n)
                         | valid_txpdo3(n) | valid_heartbeat(n))
    print(f"  driver[{n}] ({side}):")
    if not any_valid:
        print("    (nothing received yet)")
        return
    if mask & valid_rxpdo1(n):
        print(f"    RXPDO1      control_word=0x{control_word:04X}  target_torque={target_torque}")
    if mask & valid_txpdo1(n):
        print(f"    TXPDO1      status_word=0x{status_word:04X} {bits(status_word, STATUSWORD_BITS)}"
              f"  warnings={bits(warnings, WARNING_BITS)}"
              f"  error={DRIVER_ERRORS.get(error, f'0x{error:04X}')}")
    if mask & valid_txpdo2(n):
        print(f"    TXPDO2      motor_velocity={motor_velocity}"
              f"  motor_rated_current={motor_rated_current}  motor_temp={motor_temp}C")
    if mask & valid_txpdo3(n):
        print(f"    TXPDO3      driver_temp={driver_temp}C  driver_dc_voltage={driver_dc_voltage}")
    if mask & valid_heartbeat(n):
        print(f"    HEARTBEAT   state={NMT_STATES.get(heartbeat_state, f'0x{heartbeat_state:02X}')}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    fd = open_serial(port, baud)
    print(f"Listening on {port} @ {baud} baud. Ctrl-C to stop.")

    frame_num = 0
    try:
        while True:
            find_sync(fd)
            rest = read_exact(fd, FRAME_SIZE - 2)
            frame = b"\xaa\x55" + rest
            fields = struct.unpack(STRUCT_FMT, frame)

            version, mask = fields[2], fields[3]
            if version != 2:
                continue  # false sync match inside frame data; resync

            tps1, tps2 = fields[4:6]
            wsfl, wsfr, steer, fbrake = fields[6:10]
            ac1, ac2, dc1, dc2 = fields[10:14]
            (rbrake,) = fields[14:15]
            (cal,) = fields[15:16]
            node0 = fields[16:27]
            node1 = fields[27:38]

            print(f"--- frame {frame_num} ---")
            if mask & VALID_TPS:
                print(f"  TPS_DATA     tps1={tps1}  tps2={tps2}")
            if mask & VALID_FRONT:
                print(f"  FRONT_DATA   wheel_speed_fl={wsfl}  wheel_speed_fr={wsfr}"
                      f"  steering_angle={steer}  front_brake={fbrake}")
            if mask & VALID_CURRENT:
                print(f"  CURRENT_DATA ac_current_1={ac1}  ac_current_2={ac2}"
                      f"  dc_current_1={dc1}  dc_current_2={dc2}")
            if mask & VALID_REAR:
                print(f"  REAR_DATA    rear_brake={rbrake}")
            if mask & VALID_CAL:
                print(f"  CALIBRATION  type={CALIBRATION_TYPES.get(cal, cal)}")

            render_node(0, "left", node0, mask)
            render_node(1, "right", node1, mask)

            frame_num += 1
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
