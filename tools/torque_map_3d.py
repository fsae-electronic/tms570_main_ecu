"""
3D torque maps of the main ECU modes: torque request (0-1000, per mille of
rated torque) vs pedal (0-100 %) and motor speed.

    python tools/torque_map_3d.py
    python tools/torque_map_3d.py --i-rms 200 --i-dc 75 --vdc 96 --save maps.png

Uses the same functions as tools/torque_sim.py (normal_torque,
race_torque_map), which mirror s16NormalTorque() / s16RaceTorqueMap() in
app/main.c, so the maps stay in sync with the firmware logic. Traction
control isn't part of the map: it depends on wheel slip, not on pedal and
speed.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

import torque_sim as ts


def build_maps(i_rms, i_dc, vdc, tps_to_torque, n_pedal=51, n_speed=61):
    max_velocity = ts.KV_EMRAX * vdc                     # motor rpm, as in the race map
    pedal = np.linspace(0.0, 100.0, n_pedal)
    speed = np.linspace(0.0, 1.2 * max_velocity, n_speed)
    P, S = np.meshgrid(pedal, speed)

    rated_ma = int(i_rms * 1000)
    dc_dv = int(vdc * 10)
    normal = np.zeros_like(P)
    race = np.zeros_like(P)

    for i in range(P.shape[0]):
        for j in range(P.shape[1]):
            tps = ts.c_int(P[i, j])
            # Same averaging and scaling as vDriversTask (tps1 == tps2 here).
            avg_int = ((tps + tps) * tps_to_torque) // 2
            avg_float = ((tps + tps) * tps_to_torque) / 2.0
            normal[i, j] = ts.normal_torque(avg_int, rated_ma, i_dc)
            race[i, j] = ts.race_torque_map(avg_float, ts.c_int(S[i, j]), dc_dv, rated_ma, i_dc)

    return P, S, normal, race, max_velocity


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--i-rms", type=float, default=ts.MOTOR_RATED_CURRENT, help="motor RMS current at torque 1000 [A]")
    ap.add_argument("--i-dc", type=float, default=float(ts.MAX_DC_CURRENT), help="DC current limit per driver [A]")
    ap.add_argument("--vdc", type=float, default=ts.V_DC, help="DC link voltage [V]")
    ap.add_argument("--tps-to-torque", type=int, default=ts.TPS_TO_TORQUE,
                    help="pedal scale, 10 = full range (firmware TESTING value is 1)")
    ap.add_argument("--save", help="write the figure to this file instead of only showing it")
    args = ap.parse_args()

    P, S, normal, race, max_velocity = build_maps(args.i_rms, args.i_dc, args.vdc, args.tps_to_torque)
    zmax = 1000 if args.tps_to_torque >= 10 else 100 * args.tps_to_torque

    # Single-hue sequential scale (magnitude), skipping the near-white end so
    # low torque still reads against the background.
    cmap = ListedColormap(plt.get_cmap("Blues")(np.linspace(0.15, 1.0, 256)))

    fig = plt.figure(figsize=(14, 6.5))
    fig.suptitle("Torque request map  (I_RMS = %g A, I_DC = %g A, V_DC = %g V, KV = %g rpm/V)"
                 % (args.i_rms, args.i_dc, args.vdc, ts.KV_EMRAX), fontsize=11)

    panels = (
        ("Normal mode", normal),
        ("Race mode", race),
    )
    for k, (title, Z) in enumerate(panels):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        surf = ax.plot_surface(P, S, Z, cmap=cmap, vmin=0, vmax=zmax,
                               rstride=1, cstride=1, linewidth=0.2, edgecolor=(0, 0, 0, 0.15),
                               antialiased=True)
        # Motor no-load speed at this voltage: the car can't go past it.
        ax.plot([0, 100], [max_velocity, max_velocity], [0, 0], color="0.35", linewidth=1.5,
                linestyle="--", label="KV x V_DC = %d rpm" % max_velocity)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("Pedal [%]")
        ax.set_ylabel("Motor speed [rpm]")
        ax.set_zlabel("Torque request")
        ax.set_zlim(0, zmax)
        ax.view_init(elev=28, azim=-130)
        ax.legend(loc="upper left", fontsize=8)

    cax = fig.add_axes([0.93, 0.2, 0.012, 0.55])
    fig.colorbar(surf, cax=cax, label="Torque request (1000 = rated)")
    fig.subplots_adjust(left=0.02, right=0.9, wspace=0.05)

    if args.save:
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
    plt.show()


if __name__ == "__main__":
    main()
