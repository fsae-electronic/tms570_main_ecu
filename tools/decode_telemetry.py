#!/usr/bin/env python3
"""
Decode the Main ECU's telemetry stream (frame v3) on /dev/ttyUSB0.

Keep this in lockstep with the "Wire frame (v3)" section of app/telemetry.c,
which has the full byte map. Summary (little-endian):

    Header, both types (10 bytes):
        sync 0xAA 0x55, version (3), type (1 fast / 2 slow), seq (per type,
        wraps at 255), payload_len, ecu_time_ms (u32)
    Payload: payload_len bytes, layout per type (FAST_FMT / SLOW_FMT below)
    crc16 (u16): CRC-16/CCITT-FALSE over version .. end of payload

    Fast frame (type 1, 130 bytes, 10 Hz): masks, ECU state, pedals and
        chassis, torque, BMS, currents, IMU/cooling/LV, 2 x 23-byte driver
        blocks.
    Slow frame (type 2, 60 bytes, 1 Hz): firmware version, uptime, ECU
        faults, CAN error counters, motor rated current/torque.

Message catalogue / CAN IDs: see can-logger/cansniff/messages.py and the
team's "Mensajes CAN" spec doc.

No external dependencies -- configures the serial port directly via the
standard-library termios module instead of requiring pyserial.

Usage:
    python3 decode_telemetry.py [/dev/ttyUSB0] [baud]
"""
import sys
import os
import struct

VERSION = 3
TYPE_FAST, TYPE_SLOW = 1, 2

HEADER_FMT = "<2sBBBBI"  # sync, version, type, seq, payload_len, ecu_time_ms
HEADER_SIZE = struct.calcsize(HEADER_FMT)

_DRIVER_FMT = "HHHHhhhiHBBB"  # 23 bytes
_DRIVER_FIELDS = ("control_word", "status_word", "warnings", "error",
                  "target_torque", "torque_actual", "current_actual",
                  "motor_velocity", "dc_voltage", "driver_temp", "motor_temp",
                  "heartbeat_state")

FAST_FMT = ("<IIHHHBB" + "HHHHhHHHH" + "hhh" + "HhBbHHHH" + "hhhh"
            + "hhhbBH" + _DRIVER_FMT * 2)
FAST_FIELDS = (
    "valid_mask", "fresh_mask", "ecu_state", "torque_limit_reason", "ts_state",
    "calibration_type", "sdc_open_segment",
    "tps1", "tps2", "front_brake", "rear_brake", "steering_angle",
    "wheel_speed_fl", "wheel_speed_fr", "wheel_speed_rl", "wheel_speed_rr",
    "torque_request", "torque_cmd_l", "torque_cmd_r",
    "pack_voltage", "pack_current", "soc", "cell_temp_max", "cell_v_min",
    "cell_v_max", "bms_current_limit", "bms_fault",
    "dc_current_l", "dc_current_r", "ac_current_l", "ac_current_r",
    "accel_x", "accel_y", "yaw_rate", "coolant_temp", "reserved", "lv_voltage",
)
SLOW_FMT = "<BBBBIIHBBHHIBBHIIIIBBBB"
SLOW_FIELDS = (
    "fw_major", "fw_minor", "fw_patch", "fw_dirty", "git_hash", "uptime_s",
    "reset_count", "last_reset_cause", "cpu_load", "loop_overruns",
    "loop_time_max", "active_faults", "can_tx_err", "can_rx_err",
    "can_busoff_count", "motor_rated_current_l", "motor_rated_current_r",
    "motor_rated_torque_l", "motor_rated_torque_r", "cell_v_min_id",
    "cell_v_max_id", "cell_temp_max_id", "reserved",
)
assert HEADER_SIZE + struct.calcsize(FAST_FMT) + 2 == 130
assert HEADER_SIZE + struct.calcsize(SLOW_FMT) + 2 == 60
assert len(FAST_FIELDS) + 2 * len(_DRIVER_FIELDS) == len(struct.unpack(FAST_FMT, bytes(118)))
assert len(SLOW_FIELDS) == len(struct.unpack(SLOW_FMT, bytes(48)))

VALID_BITS = {
    0: "TPS", 1: "FRONT", 2: "CURRENT", 3: "REAR", 4: "CAL",
    5: "L_RXPDO1", 6: "L_TXPDO1", 7: "L_TXPDO2", 8: "L_TXPDO3", 9: "L_HB",
    10: "R_RXPDO1", 11: "R_TXPDO1", 12: "R_TXPDO2", 13: "R_TXPDO3", 14: "R_HB",
    15: "BMS", 16: "REAR_WHEELS", 17: "IMU", 18: "COOLANT", 19: "LV", 20: "TS",
}
ECU_STATE_BITS = {
    2: "DriveEnabled", 3: "ReadyToDrive", 4: "TC_On", 5: "TC_CutL",
    6: "TC_CutR", 7: "APPS_Implausible", 8: "APPS_Brake", 9: "BrakePressed",
    10: "SafeMode",
}
TORQUE_LIMIT_BITS = {
    0: "DriveDisabled", 1: "APPS", 2: "APPS_Brake", 3: "Traction", 4: "Power",
    5: "BMS_Current", 6: "MotorTemp", 7: "DriverTemp", 8: "CellTemp",
    9: "CellUndervoltage", 10: "Speed", 11: "DriverNotEnabled",
}
TS_STATE_BITS = {
    0: "SDC", 1: "AIR+", 2: "AIR-", 3: "PrechargeRelay", 4: "PrechargeDone",
    5: "IMD_OK", 6: "AMS_OK", 7: "BSPD_OK", 8: "TS_Active", 9: "TSMS",
}
ACTIVE_FAULT_BITS = {
    0: "TPS_Timeout", 1: "Front_Timeout", 2: "Rear_Timeout",
    3: "SensorImplausible", 4: "APPS_Brake",
}
RESET_CAUSES = {0: "unknown", 1: "power-on", 2: "watchdog", 3: "external",
                4: "software", 5: "oscillator", 6: "CPU (ESM)", 7: "brown-out"}

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


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


assert crc16_ccitt_false(b"123456789") == 0x29B1


def hex4(value):
    return f"0x{value:04X}"


def hex2(value):
    return f"0x{value:02X}"


def bits(value, table):
    names = [name for bit, name in sorted(table.items()) if value >> bit & 1]
    return "|".join(names) if names else "-"


def parse_frame(frame):
    """Decode one complete frame (sync included). Returns (header, fields)
    or raises ValueError if the CRC or the layout doesn't match."""
    _, version, ftype, seq, payload_len, time_ms = struct.unpack_from(HEADER_FMT, frame)
    end = HEADER_SIZE + payload_len
    (crc,) = struct.unpack_from("<H", frame, end)
    if crc != crc16_ccitt_false(frame[2:end]):
        raise ValueError("bad CRC")
    header = {"version": version, "type": ftype, "seq": seq, "ecu_time_ms": time_ms}
    payload = frame[HEADER_SIZE:end]

    if ftype == TYPE_FAST and payload_len == struct.calcsize(FAST_FMT):
        values = struct.unpack(FAST_FMT, payload)
        n = len(FAST_FIELDS)
        d = len(_DRIVER_FIELDS)
        fields = dict(zip(FAST_FIELDS, values[:n]))
        fields["driver"] = [dict(zip(_DRIVER_FIELDS, values[n + i * d:n + (i + 1) * d]))
                            for i in range(2)]
        return header, fields
    if ftype == TYPE_SLOW and payload_len == struct.calcsize(SLOW_FMT):
        return header, dict(zip(SLOW_FIELDS, struct.unpack(SLOW_FMT, payload)))
    return header, None  # unknown type: skipped thanks to payload_len


def render_fast(f):
    valid, fresh = f["valid_mask"], f["fresh_mask"]
    state = f["ecu_state"]
    mode = "race" if state & 0x3 == 1 else "normal"
    print(f"  valid={bits(valid, VALID_BITS)}")
    print(f"  stale={bits(valid & ~fresh, VALID_BITS)}")
    print(f"  STATE        mode={mode}  {bits(state, ECU_STATE_BITS)}"
          f"  limit={bits(f['torque_limit_reason'], TORQUE_LIMIT_BITS)}"
          f"  cal={CALIBRATION_TYPES.get(f['calibration_type'], f['calibration_type'])}")
    if valid & (1 << 20):
        print(f"  TS           {bits(f['ts_state'], TS_STATE_BITS)}"
              f"  sdc_open_segment={f['sdc_open_segment']}")
    print(f"  PEDALS       tps1={f['tps1'] / 10:.1f}%  tps2={f['tps2'] / 10:.1f}%"
          f"  brake F/R={f['front_brake']}/{f['rear_brake']} PSI"
          f"  steering={f['steering_angle']}")
    print(f"  WHEELS       fl={f['wheel_speed_fl'] / 10:.1f}  fr={f['wheel_speed_fr'] / 10:.1f}"
          f"  rl={f['wheel_speed_rl'] / 10:.1f}  rr={f['wheel_speed_rr'] / 10:.1f} rpm")
    print(f"  TORQUE       request={f['torque_request']}  cmd L/R={f['torque_cmd_l']}/{f['torque_cmd_r']}")
    if valid & (1 << 15):
        print(f"  BMS          {f['pack_voltage'] / 10:.1f} V  {f['pack_current'] / 10:.1f} A"
              f"  SoC={f['soc']}%  cells {f['cell_v_min']}-{f['cell_v_max']} mV"
              f"  Tmax={f['cell_temp_max']}C  limit={f['bms_current_limit'] / 10:.1f} A"
              f"  fault=0x{f['bms_fault']:04X}")
    print(f"  CURRENTS     dc L/R={f['dc_current_l']}/{f['dc_current_r']}"
          f"  ac L/R={f['ac_current_l']}/{f['ac_current_r']}")
    if valid & (1 << 17):
        print(f"  IMU          ax={f['accel_x'] / 1000:.3f} g  ay={f['accel_y'] / 1000:.3f} g"
              f"  yaw={f['yaw_rate'] / 100:.2f} deg/s")
    if valid & (1 << 18):
        print(f"  COOLANT      {f['coolant_temp']}C")
    if valid & (1 << 19):
        print(f"  LV           {f['lv_voltage'] / 100:.2f} V")
    for n, side in enumerate(("left", "right")):
        d = f["driver"][n]
        print(f"  driver[{n}] ({side}):"
              f" cw=0x{d['control_word']:04X} sw=0x{d['status_word']:04X} {bits(d['status_word'], STATUSWORD_BITS)}"
              f"  warn={bits(d['warnings'], WARNING_BITS)}"
              f"  err={DRIVER_ERRORS.get(d['error'], hex4(d['error']))}")
        print(f"               target={d['target_torque']}  actual={d['torque_actual']}"
              f"  current={d['current_actual']}  vel={d['motor_velocity']} rpm"
              f"  {d['dc_voltage'] / 10:.1f} V  Tdrv={d['driver_temp']}C  Tmot={d['motor_temp']}C"
              f"  NMT={NMT_STATES.get(d['heartbeat_state'], hex2(d['heartbeat_state']))}")


def render_slow(f):
    print(f"  FW           {f['fw_major']}.{f['fw_minor']}.{f['fw_patch']}"
          f"{'-dirty' if f['fw_dirty'] else ''}  git={f['git_hash']:08x}"
          f"  uptime={f['uptime_s']} s  resets={f['reset_count']}"
          f"  last_reset={RESET_CAUSES.get(f['last_reset_cause'], f['last_reset_cause'])}")
    print(f"  ECU          cpu={f['cpu_load']}%  overruns={f['loop_overruns']}"
          f"  loop_max={f['loop_time_max']} us  faults={bits(f['active_faults'], ACTIVE_FAULT_BITS)}")
    print(f"  CAN          TEC={f['can_tx_err']}  REC={f['can_rx_err']}  busoff={f['can_busoff_count']}")
    print(f"  MOTORS       rated current L/R={f['motor_rated_current_l']}/{f['motor_rated_current_r']} mA"
          f"  rated torque L/R={f['motor_rated_torque_l']}/{f['motor_rated_torque_r']} mNm")


def open_serial(port, baud):
    import termios

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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    fd = open_serial(port, baud)
    print(f"Listening on {port} @ {baud} baud. Ctrl-C to stop.")

    last_seq = {}
    lost = {TYPE_FAST: 0, TYPE_SLOW: 0}
    try:
        while True:
            find_sync(fd)
            rest = read_exact(fd, HEADER_SIZE - 2)
            if rest[0] != VERSION:
                continue  # false sync match inside frame data; resync
            payload_len = rest[3]
            frame = b"\xaa\x55" + rest + read_exact(fd, payload_len + 2)
            try:
                header, fields = parse_frame(frame)
            except ValueError:
                continue  # bad CRC: false sync or corrupted frame
            if fields is None:
                continue

            ftype, seq = header["type"], header["seq"]
            if ftype in last_seq:
                lost[ftype] += (seq - last_seq[ftype] - 1) & 0xFF
            last_seq[ftype] = seq

            name = "fast" if ftype == TYPE_FAST else "slow"
            print(f"--- {name} seq={seq} t={header['ecu_time_ms'] / 1000:.3f} s"
                  f"  (lost fast={lost[TYPE_FAST]} slow={lost[TYPE_SLOW]}) ---")
            if ftype == TYPE_FAST:
                render_fast(fields)
            else:
                render_slow(fields)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
