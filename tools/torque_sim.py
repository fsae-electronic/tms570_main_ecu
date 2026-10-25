"""
Interactive simulator of the main ECU torque path (app/main.c + app/traction.c).

    python tools/torque_sim.py

What's simulated
----------------
ECU (every 10 ms, same logic and integer casts as the firmware):
  * Normal mode: s16NormalTorque() -- pedal 0-100 % -> 0 .. 1000 *
                 MAX_DC_CURRENT / I_rated, so the phase current never
                 exceeds the battery DC limit (at top speed I_phase_rms ~ I_dc)
  * Race mode:   s16RaceTorqueMap() per motor (KV_EMRAX, MAX_DC_CURRENT,
                 driver_dc_voltage, motor_rated_current)
  * Traction control: traction_apply() -- slip ratio + PI per rear motor
  * Torque is sent to the drives every 10 ms (TORQUE_SEND_PERIOD_MS in
    drivers.c, same as DRIVERS_PERIOD_MS); in between the drive holds the
    last value.
Car (1D longitudinal, simple physics just to have something to control):
  * Two rear motors through a fixed reduction, front wheels free rolling.
  * Tire force from a simplified Pacejka curve, one mu per rear wheel so a
    split-mu surface can be tried.
  * Motor torque falls off near no-load speed (KV_EMRAX * V_DC).
  * "Bench" mode: car held still (fronts at 0), rears in the air with only
    friction -- the test-bench case.

Units shown are the ones the ECU sees: front = wheel RPM from 0x502,
rear = motor_velocity / REAR_DIVISOR (xTelemetryData.rear.wheel_speed_rl/rr).

The vehicle numbers below are placeholders -- set them to the real car.
"""

import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

# ---- Firmware constants (keep in sync with main.c / traction.h / drivers.c) --

MAX_DC_CURRENT = 75          # A      (main.c) -- default of the 'I DC max' slider
KV_EMRAX = 22.0              # RPM/V  (main.c)

TC_TARGET_SLIP = 0.15        # traction.h
TC_KP = 2.0
TC_KI = 10.0
TC_MIN_WHEEL_SPEED = 30.0

TPS_TO_TORQUE = 10           # main.c: TPS 0-100 % -> torque 0-1000 per mille
DRIVERS_PERIOD_MS = 10       # main.c
TORQUE_SEND_PERIOD_MS = 10   # drivers.c

# Rear wheel speed = motor_velocity / GEAR_RATIO (main.c), wheel RPM like
# the fronts.
GEAR_RATIO = 2.0             # motor turns per wheel turn (main.c)
REAR_DIVISOR = GEAR_RATIO

# ---- Vehicle model (placeholders) -------------------------------------------

MASS = 350.0                 # kg, car + driver
WHEEL_RADIUS = 0.23          # m
REAR_WEIGHT_FRACTION = 0.55
G = 9.81
CDA = 1.0                    # m^2, drag area
RHO = 1.2                    # kg/m^3
CRR = 0.015                  # rolling resistance
WHEEL_INERTIA = 0.3          # kg m^2, rear wheel + hub
MOTOR_INERTIA = 0.0144         # kg m^2, rotor (reflected through GEAR_RATIO^2)
V_DC = 96.0                  # V, 29 LiFePO4 cells ~3.3 V
KT = 0.32                    # Nm / A_rms, motor torque constant
# Motor rated current: max RMS phase current, sent when torque = 1000.
# Default of the "I RMS motor" slider. Rated torque (torque request 1000)
# = KT * motor rated current, computed live in Car.step().
MOTOR_RATED_CURRENT = 200.0  # A_rms
BENCH_FRICTION_NM = 2.0      # wheel-side friction on the bench
BENCH_VISCOUS = 0.05         # Nm per rad/s on the bench

PACEJKA_B = 10.0
PACEJKA_C = 1.9

SIM_DT = 0.0002              # s, physics step
FRAME_S = 0.05               # s, sim time per animation frame (real time)
WINDOW_S = 10.0              # s, visible history


# ---- Firmware logic, ported 1:1 ---------------------------------------------

def c_int(x):
    """C float -> integer cast (truncates toward zero)."""
    return int(x)


def normal_torque(sensor_value, motor_rated_current, max_dc_current=MAX_DC_CURRENT):
    """Normal mode, linear with the pedal but capped so the phase current
    (motor_rated_current at torque 1000, in mA) stays at MAX_DC_CURRENT:
    torque_max = 1000 * MAX_DC_CURRENT / I_rated. sensor_value is 0-1000.
    E.g. I_rated = 200 A -> torque_max = 250."""
    if motor_rated_current == 0:
        return 0                     # no TXPDO2 yet -> no current reference
    torque_max = c_int((1000 * max_dc_current * 1000) // motor_rated_current)
    return c_int(sensor_value * min(torque_max, 1000) / 1000)


def race_torque_map(sensor_value, motor_velocity, driver_dc_voltage, motor_rated_current,
                    max_dc_current=MAX_DC_CURRENT):
    """s16RaceTorqueMap() in main.c."""
    velocity_value = float(abs(motor_velocity))
    max_velocity = KV_EMRAX * (driver_dc_voltage / 10.0)

    if motor_rated_current == 0:
        mmin = math.inf
    else:
        mmin = max_dc_current / (motor_rated_current / 1000.0)

    if sensor_value > 10.0:
        m = mmin * (1000.0 / sensor_value)
    else:
        m = 100.0

    if velocity_value > m * max_velocity:
        tps_value = c_int((mmin * 1000.0 * max_velocity) / velocity_value)
    else:
        tps_value = c_int(sensor_value)

    return tps_value


class TractionControl:
    """app/traction.c."""

    def __init__(self):
        self.integral = [0.0, 0.0]
        self.slip = [0.0, 0.0]
        self.cut = [0.0, 0.0]

    def reset_wheel(self, n):
        self.integral[n] = 0.0
        self.slip[n] = 0.0
        self.cut[n] = 0.0

    def reset(self):
        self.reset_wheel(0)
        self.reset_wheel(1)

    def apply(self, front, rear, torque, dt_s):
        for n in range(2):
            if rear[n] < TC_MIN_WHEEL_SPEED:
                self.reset_wheel(n)
                continue

            self.slip[n] = (rear[n] - front[n]) / rear[n]
            error = self.slip[n] - TC_TARGET_SLIP

            self.integral[n] = min(max(self.integral[n] + TC_KI * error * dt_s, 0.0), 1.0)
            self.cut[n] = min(max(TC_KP * error + self.integral[n], 0.0), 1.0)

            torque[n] = c_int(torque[n] * (1.0 - self.cut[n]))
        return torque


# ---- Car model ----------------------------------------------------------------

def tire_force(kappa, mu, fz):
    return mu * fz * math.sin(PACEJKA_C * math.atan(PACEJKA_B * kappa))


class Car:
    def __init__(self):
        self.reset()

    def reset(self):
        self.v = 0.0                   # m/s
        self.w_rear = [0.0, 0.0]       # rad/s, wheel
        self.kappa = [0.0, 0.0]

    def motor_rpm(self, n):
        return self.w_rear[n] * GEAR_RATIO * 60.0 / (2.0 * math.pi)

    def front_rpm(self):
        return self.v / WHEEL_RADIUS * 60.0 / (2.0 * math.pi)

    def step(self, torque_req, mu, bench, dt, rated_current_a):
        inertia = WHEEL_INERTIA + MOTOR_INERTIA * GEAR_RATIO ** 2
        fz = MASS * G * REAR_WEIGHT_FRACTION / 2.0
        no_load_rpm = KV_EMRAX * V_DC
        fx_total = 0.0

        for n in range(2):
            # Motor torque, fading out over the last 10 % before no-load speed.
            rpm = self.motor_rpm(n)
            fade = min(max((no_load_rpm - rpm) / (0.1 * no_load_rpm), 0.0), 1.0)
            rated_torque_nm = KT * rated_current_a          # Nm at torque request 1000
            wheel_torque = torque_req[n] / 1000.0 * rated_torque_nm * fade * GEAR_RATIO

            if bench:
                w = self.w_rear[n]
                resist = (BENCH_FRICTION_NM * (1.0 if w > 0.0 else 0.0)) + BENCH_VISCOUS * w
                self.w_rear[n] = max(0.0, w + (wheel_torque - resist) / inertia * dt)
                self.kappa[n] = 1.0 if self.w_rear[n] > 0.0 else 0.0
                continue

            ground = self.w_rear[n] * WHEEL_RADIUS
            self.kappa[n] = (ground - self.v) / max(ground, self.v, 0.5)
            fx = tire_force(self.kappa[n], mu[n], fz)
            fx_total += fx
            self.w_rear[n] = max(0.0, self.w_rear[n] + (wheel_torque - fx * WHEEL_RADIUS) / inertia * dt)

        if bench:
            self.v = 0.0
            return

        resist = 0.5 * RHO * CDA * self.v ** 2 + (CRR * MASS * G if self.v > 0.0 else 0.0)
        self.v = max(0.0, self.v + (fx_total - resist) / MASS * dt)


# ---- Simulation state --------------------------------------------------------

class Sim:
    def __init__(self):
        self.race_mode = False
        self.traction_on = True        # firmware default
        self.bench = False
        self.paused = False
        self.tps = 0.0
        self.mu = [1.2, 1.2]
        self.rated_current_a = MOTOR_RATED_CURRENT  # motor RMS phase current at torque 1000
        self.max_dc_a = float(MAX_DC_CURRENT)  # battery DC current limit per driver
        self.car = Car()
        self.tc = TractionControl()
        self.reset()

    def reset(self):
        self.car.reset()
        self.tc.reset()
        self.t = 0.0
        self.next_ecu = 0.0
        self.ecu_ticks = 0
        self.drive_torque = [0, 0]     # what the drives are applying
        self.torque_cmd = [0, 0]       # xTelemetryData.torque_cmd
        self.hist = {k: [] for k in ("t", "fl", "fr", "rl", "rr", "pedal",
                                      "tq_l", "tq_r", "slip_l", "slip_r",
                                      "cut_l", "cut_r")}

    def ecu_step(self):
        # What the ECU sees on the bus.
        front_rpm = c_int(self.car.front_rpm())          # uint16 in 0x502
        motor_velocity = [c_int(self.car.motor_rpm(n)) for n in range(2)]
        rear = [c_int(mv / REAR_DIVISOR) for mv in motor_velocity]
        tps1 = tps2 = c_int(self.tps)

        if self.race_mode:
            avg = ((tps1 + tps2) * TPS_TO_TORQUE) / 2.0
            torque = [race_torque_map(avg, motor_velocity[n], int(V_DC * 10),
                                      int(self.rated_current_a * 1000), self.max_dc_a)
                      for n in range(2)]
        else:
            avg = ((tps1 + tps2) * TPS_TO_TORQUE) // 2
            torque = [normal_torque(avg, int(self.rated_current_a * 1000), self.max_dc_a)
                      for _ in range(2)]

        if self.traction_on:
            torque = self.tc.apply([float(front_rpm)] * 2, [float(r) for r in rear],
                                   torque, DRIVERS_PERIOD_MS / 1000.0)
        else:
            self.tc.reset()

        self.torque_cmd = torque

        # run_drivers(): torque only goes out every TORQUE_SEND_PERIOD_MS.
        if self.ecu_ticks % (TORQUE_SEND_PERIOD_MS // DRIVERS_PERIOD_MS) == 0:
            self.drive_torque = list(torque)
        self.ecu_ticks += 1

        h = self.hist
        h["t"].append(self.t)
        h["fl"].append(front_rpm)
        h["fr"].append(front_rpm)
        h["rl"].append(rear[0])
        h["rr"].append(rear[1])
        h["pedal"].append(self.tps * TPS_TO_TORQUE)
        h["tq_l"].append(torque[0])
        h["tq_r"].append(torque[1])
        h["slip_l"].append(self.tc.slip[0])
        h["slip_r"].append(self.tc.slip[1])
        h["cut_l"].append(self.tc.cut[0])
        h["cut_r"].append(self.tc.cut[1])

    def advance(self, seconds):
        end = self.t + seconds
        while self.t < end:
            if self.t >= self.next_ecu:
                self.ecu_step()
                self.next_ecu += DRIVERS_PERIOD_MS / 1000.0
            self.car.step(self.drive_torque, self.mu, self.bench, SIM_DT, self.rated_current_a)
            self.t += SIM_DT

        # Drop history older than the visible window.
        cutoff = self.t - WINDOW_S
        while self.hist["t"] and self.hist["t"][0] < cutoff:
            for v in self.hist.values():
                v.pop(0)


# ---- UI ------------------------------------------------------------------------

def main():
    sim = Sim()

    fig = plt.figure(figsize=(13, 9))
    fig.canvas.manager.set_window_title("Main ECU torque simulator")
    fig.subplots_adjust(left=0.07, right=0.97, top=0.95, bottom=0.30, hspace=0.35)

    ax_speed = fig.add_subplot(3, 1, 1)
    ax_torque = fig.add_subplot(3, 1, 2, sharex=ax_speed)
    ax_slip = fig.add_subplot(3, 1, 3, sharex=ax_speed)

    l_fl, = ax_speed.plot([], [], label="FL (0x502)", color="tab:blue")
    l_fr, = ax_speed.plot([], [], label="FR (0x502)", color="tab:cyan", linestyle="--")
    l_rl, = ax_speed.plot([], [], label="RL (motor/%g)" % REAR_DIVISOR, color="tab:red")
    l_rr, = ax_speed.plot([], [], label="RR (motor/%g)" % REAR_DIVISOR, color="tab:orange", linestyle="--")
    ax_speed.set_ylabel("Wheel speed [rpm]")
    ax_speed.legend(loc="upper left", fontsize=8)
    ax_speed.grid(True, alpha=0.3)
    info = ax_speed.text(0.99, 0.95, "", transform=ax_speed.transAxes,
                         ha="right", va="top", fontsize=9, family="monospace")

    l_pedal, = ax_torque.plot([], [], label="Pedal (TPS avg x%d)" % TPS_TO_TORQUE, color="gray", linestyle=":")
    l_tql, = ax_torque.plot([], [], label="torque_cmd[0] L", color="tab:red")
    l_tqr, = ax_torque.plot([], [], label="torque_cmd[1] R", color="tab:orange", linestyle="--")
    ax_torque.set_ylabel("Torque request [0-1000]")
    ax_torque.set_ylim(-50, 1100)
    ax_torque.legend(loc="upper left", fontsize=8)
    ax_torque.grid(True, alpha=0.3)

    l_sl, = ax_slip.plot([], [], label="slip L", color="tab:red")
    l_sr, = ax_slip.plot([], [], label="slip R", color="tab:orange", linestyle="--")
    l_cl, = ax_slip.plot([], [], label="TC cut L", color="tab:purple")
    l_cr, = ax_slip.plot([], [], label="TC cut R", color="tab:pink", linestyle="--")
    ax_slip.axhline(TC_TARGET_SLIP, color="k", linewidth=0.8, linestyle=":", label="target slip")
    ax_slip.set_ylabel("Slip / cut")
    ax_slip.set_xlabel("Time [s]")
    ax_slip.set_ylim(-0.1, 1.1)
    ax_slip.legend(loc="upper left", fontsize=8, ncol=5)
    ax_slip.grid(True, alpha=0.3)

    # Sliders
    s_tps = Slider(fig.add_axes([0.12, 0.20, 0.55, 0.025]), "TPS [%]", 0, 100, valinit=0, valstep=1)
    s_mul = Slider(fig.add_axes([0.12, 0.16, 0.55, 0.025]), "mu left", 0.1, 1.5, valinit=sim.mu[0])
    s_mur = Slider(fig.add_axes([0.12, 0.12, 0.55, 0.025]), "mu right", 0.1, 1.5, valinit=sim.mu[1])
    s_cur = Slider(fig.add_axes([0.12, 0.08, 0.55, 0.025]), "I RMS motor [A]", 10, 600,
                   valinit=sim.rated_current_a, valstep=5)
    s_dc = Slider(fig.add_axes([0.12, 0.04, 0.55, 0.025]), "I DC driver [A]", 5, 200,
                  valinit=sim.max_dc_a, valstep=1)

    def on_tps(val):
        sim.tps = val

    def on_mu(_):
        sim.mu = [s_mul.val, s_mur.val]

    def on_cur(val):
        sim.rated_current_a = val

    def on_dc(val):
        sim.max_dc_a = val

    s_tps.on_changed(on_tps)
    s_mul.on_changed(on_mu)
    s_mur.on_changed(on_mu)
    s_cur.on_changed(on_cur)
    s_dc.on_changed(on_dc)

    # Buttons (same toggles as the dashboard: mode 0xB/0xC, traction 9/0xA)
    b_mode = Button(fig.add_axes([0.74, 0.18, 0.11, 0.05]), "")
    b_tc = Button(fig.add_axes([0.86, 0.18, 0.11, 0.05]), "")
    b_bench = Button(fig.add_axes([0.74, 0.11, 0.11, 0.05]), "")
    b_reset = Button(fig.add_axes([0.86, 0.11, 0.11, 0.05]), "Reset")
    b_pause = Button(fig.add_axes([0.74, 0.04, 0.23, 0.05]), "")

    def refresh_labels():
        b_mode.label.set_text("Mode: RACE" if sim.race_mode else "Mode: NORMAL")
        b_tc.label.set_text("TC: ON" if sim.traction_on else "TC: OFF")
        b_bench.label.set_text("Bench: ON" if sim.bench else "Bench: OFF")
        b_pause.label.set_text("Resume" if sim.paused else "Pause")
        fig.canvas.draw_idle()

    def on_mode(_):
        sim.race_mode = not sim.race_mode
        refresh_labels()

    def on_tc(_):
        sim.traction_on = not sim.traction_on
        refresh_labels()

    def on_bench(_):
        sim.bench = not sim.bench
        sim.reset()
        refresh_labels()

    def on_reset(_):
        sim.reset()

    b_mode.on_clicked(on_mode)
    b_tc.on_clicked(on_tc)
    b_bench.on_clicked(on_bench)
    def on_pause(_):
        sim.paused = not sim.paused
        refresh_labels()

    b_reset.on_clicked(on_reset)
    b_pause.on_clicked(on_pause)
    refresh_labels()

    def update(_):
        # Paused: time stops and the plots freeze (buttons and sliders still
        # work and take effect on resume).
        if sim.paused:
            return ()
        sim.advance(FRAME_S)
        h = sim.hist
        t = h["t"]

        l_fl.set_data(t, h["fl"])
        l_fr.set_data(t, h["fr"])
        l_rl.set_data(t, h["rl"])
        l_rr.set_data(t, h["rr"])
        l_pedal.set_data(t, h["pedal"])
        l_tql.set_data(t, h["tq_l"])
        l_tqr.set_data(t, h["tq_r"])
        l_sl.set_data(t, h["slip_l"])
        l_sr.set_data(t, h["slip_r"])
        l_cl.set_data(t, h["cut_l"])
        l_cr.set_data(t, h["cut_r"])

        ax_speed.set_xlim(max(0.0, sim.t - WINDOW_S), max(WINDOW_S, sim.t))
        top = max([1.0] + h["fl"] + h["rl"] + h["rr"])
        ax_speed.set_ylim(-1, top * 1.1)

        info.set_text("v = %5.1f km/h   motor L/R = %5d / %5d rpm"
                      % (sim.car.v * 3.6, c_int(sim.car.motor_rpm(0)), c_int(sim.car.motor_rpm(1))))
        return ()

    anim = FuncAnimation(fig, update, interval=int(FRAME_S * 1000), cache_frame_data=False)
    # Keep references so nothing is garbage collected while the window is open.
    fig._anim = anim
    fig._widgets = {"tps": s_tps, "mu_l": s_mul, "mu_r": s_mur, "current": s_cur, "dc": s_dc,
                    "mode": b_mode, "tc": b_tc, "bench": b_bench, "reset": b_reset,
                    "pause": b_pause}
    plt.show()


if __name__ == "__main__":
    main()
