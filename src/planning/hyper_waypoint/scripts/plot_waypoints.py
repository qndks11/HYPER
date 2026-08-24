#!/usr/bin/env python3
"""Plot a waypoint_recorder_node CSV (idx,x,y,yaw,frame_id,gps_lat,gps_lon,gps_status,
gps_odom_x,gps_odom_y) as an x-y path, overlaid with two upstream stages -- the raw GPS
fix (converted to local meters) and navsat_transform's output (/odometry/gps, already in
map-frame meters, pre-fusion) -- so a jump can be traced to the GPS sensor itself,
navsat_transform's conversion, or ekf_global's fusion.

Usage:
    python3 plot_waypoints.py /path/to/waypoints.csv [--jump-threshold 1.0]
"""
import argparse
import csv
import math
import sys

import matplotlib.pyplot as plt

EARTH_RADIUS_M = 6378137.0


def load_waypoints(csv_path):
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def latlon_to_local_xy(lat, lon, lat0, lon0):
    """Equirectangular approximation, good enough to compare against map x/y for a
    small-area run (matches ENU when heading/declination/yaw_offset are 0, as in the
    'sim' datum)."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlon * math.cos(math.radians(lat0)) * EARTH_RADIUS_M
    y = dlat * EARTH_RADIUS_M
    return x, y


def find_jumps(points, threshold_m):
    jumps = []
    prev = None
    for i, p in points:
        if prev is not None:
            dist = math.hypot(p[0] - prev[1][0], p[1] - prev[1][1])
            if dist > threshold_m:
                jumps.append((prev[0], i, dist))
        prev = (i, p)
    return jumps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="waypoint_recorder_node output CSV")
    parser.add_argument(
        "--jump-threshold", type=float, default=1.0,
        help="mark consecutive-row jumps bigger than this many meters (default: 1.0)")
    args = parser.parse_args()

    rows = load_waypoints(args.csv_path)
    if not rows:
        sys.exit(f"No rows found in '{args.csv_path}'")

    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]

    gps_points = []
    lat0 = lon0 = None
    for i, r in enumerate(rows):
        lat_s, lon_s = r.get("gps_lat", ""), r.get("gps_lon", "")
        if not lat_s or not lon_s:
            continue
        lat, lon = float(lat_s), float(lon_s)
        if lat0 is None:
            lat0, lon0 = lat, lon
        gx, gy = latlon_to_local_xy(lat, lon, lat0, lon0)
        gps_points.append((i, (gx, gy)))

    gps_odom_points = []
    for i, r in enumerate(rows):
        x_s, y_s = r.get("gps_odom_x", ""), r.get("gps_odom_y", "")
        if not x_s or not y_s:
            continue
        gps_odom_points.append((i, (float(x_s), float(y_s))))

    odom_points = []
    odom_dead_reckoned = False
    if rows and rows[0].get("odom_x"):
        # Recorded directly from /odom's own pose (added to waypoint_recorder_node --
        # rebuild+re-record to get this for older CSVs).
        for i, r in enumerate(rows):
            x_s, y_s = r.get("odom_x", ""), r.get("odom_y", "")
            if not x_s or not y_s:
                continue
            odom_points.append((i, (float(x_s), float(y_s))))
    elif any(r.get("odom_vx") for r in rows):
        # Older CSV recorded before odom_x/odom_y existed: approximate the path by
        # dead-reckoning the logged odom_vx/vy/vyaw between rows (Euler integration over
        # each row's real dt, using odom_yaw if present else the fused yaw as heading).
        odom_dead_reckoned = True
        x, y = 0.0, 0.0
        yaw = float(rows[0]["yaw"]) if rows[0].get("yaw") else 0.0
        prev_stamp = None
        for i, r in enumerate(rows):
            vx_s, vy_s, vyaw_s = r.get("odom_vx", ""), r.get("odom_vy", ""), r.get("odom_vyaw", "")
            stamp_s = r.get("stamp_sec", "")
            if not (vx_s and vy_s and stamp_s):
                continue
            stamp = float(stamp_s)
            if prev_stamp is not None:
                dt = max(0.0, stamp - prev_stamp)
                vx, vy, vyaw = float(vx_s), float(vy_s), float(vyaw_s)
                x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
                y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
                yaw += vyaw * dt
            prev_stamp = stamp
            odom_points.append((i, (x, y)))

    fused_points = list(enumerate(zip(xs, ys)))
    fused_jumps = find_jumps(fused_points, args.jump_threshold)
    gps_jumps = find_jumps(gps_points, args.jump_threshold)
    gps_odom_jumps = find_jumps(gps_odom_points, args.jump_threshold)

    stamps = [float(r["stamp_sec"]) if r.get("stamp_sec") else None for r in rows]
    fused_yaws = [float(r["yaw"]) if r.get("yaw") else None for r in rows]
    imu_yaws = [float(r["imu_yaw"]) if r.get("imu_yaw") else None for r in rows]
    imu_wz = [float(r["imu_wz"]) if r.get("imu_wz") else None for r in rows]
    imu_ax = [float(r["imu_ax"]) if r.get("imu_ax") else None for r in rows]
    imu_ay = [float(r["imu_ay"]) if r.get("imu_ay") else None for r in rows]

    print(f"fused (odometry/filtered_map) jumps > {args.jump_threshold} m:")
    for i0, i1, dist in fused_jumps:
        stamp_note = ""
        if stamps[i0] is not None and stamps[i1] is not None:
            stamp_note = (f"  (stamp {stamps[i0]:.3f}s -> {stamps[i1]:.3f}s -- check "
                           f"*.diag.log around this window)")
        print(f"  idx {i0} -> {i1}: {dist:.2f} m{stamp_note}")

    if not gps_points:
        print("no gps_lat/gps_lon data in this CSV -- re-record with the updated "
              "waypoint_recorder_node to capture GPS alongside the fused pose.")
    else:
        print(f"raw gps/fix jumps > {args.jump_threshold} m:")
        for i0, i1, dist in gps_jumps:
            print(f"  idx {i0} -> {i1}: {dist:.2f} m")

    if not gps_odom_points:
        print("no gps_odom_x/gps_odom_y data in this CSV -- re-record with the updated "
              "waypoint_recorder_node to capture navsat_transform's output.")
    else:
        print(f"navsat_transform (odometry/gps) jumps > {args.jump_threshold} m:")
        for i0, i1, dist in gps_odom_jumps:
            print(f"  idx {i0} -> {i1}: {dist:.2f} m")

    if gps_points and gps_odom_points:
        if gps_jumps:
            print("=> raw gps/fix itself jumps: look at the Gazebo navsat plugin/sensor.")
        elif gps_odom_jumps:
            print("=> gps/fix is smooth but navsat_transform's output jumps: look at "
                  "navsat_transform (datum/yaw handling), not the GPS sensor or ekf_global.")
        elif fused_jumps:
            print("=> gps/fix and navsat_transform output are both smooth but the fused "
                  "pose jumps: look at ekf_global (process/initial covariance tuning, "
                  "outlier rejection).")

    cov_idx, cov_xx, cov_yy = [], [], []
    for i, r in enumerate(rows):
        cxx_s, cyy_s = r.get("cov_xx", ""), r.get("cov_yy", "")
        if not cxx_s or not cyy_s:
            continue
        cov_idx.append(i)
        cov_xx.append(float(cxx_s))
        cov_yy.append(float(cyy_s))
    if cov_idx:
        print("cov_xx/cov_yy right before each fused jump (watch for a spike leading "
              "into the jump -- that means ekf_global was losing confidence / rejecting "
              "GPS beforehand, then snapped back once it re-accepted a fix):")
        for i0, i1, dist in fused_jumps:
            if i0 in cov_idx:
                j = cov_idx.index(i0)
                print(f"  idx {i0} (before {dist:.1f} m jump): cov_xx={cov_xx[j]:.4g}, "
                      f"cov_yy={cov_yy[j]:.4g}")

    raw_fields = ["odom_vx", "odom_vy", "odom_vyaw", "imu_wz", "imu_ax", "imu_ay", "imu_yaw"]
    if fused_jumps and any(r.get("odom_vx") for r in rows):
        print("raw /odom and /imu readings at the row before and at each fused jump "
              "(look for a spike/NaN -- that points at the sensor/plugin feeding "
              "ekf_global's predict step, not GPS or covariance tuning):")
        for i0, i1, dist in fused_jumps:
            def fmt(i):
                r = rows[i]
                vals = [r.get(f, "") for f in raw_fields]
                if not all(vals):
                    return "(no raw odom/imu data recorded yet)"
                return ", ".join(f"{f}={v}" for f, v in zip(raw_fields, vals))
            print(f"  idx {i0} (before {dist:.1f} m jump): {fmt(i0)}")
            print(f"  idx {i1} (at jump):              {fmt(i1)}")

    fig, ((ax, ax_cov), (ax_yaw, ax_imu)) = plt.subplots(2, 2, figsize=(16, 14))
    ax.plot(xs, ys, "-", color="tab:blue", linewidth=1, marker=".", markersize=3,
            label="fused (odometry/filtered_map)")
    if gps_points:
        gxs = [p[0] for _, p in gps_points]
        gys = [p[1] for _, p in gps_points]
        ax.plot(gxs, gys, "-", color="tab:orange", linewidth=1, marker=".",
                markersize=3, alpha=0.8, label="gps/fix (local, relative to first fix)")
    if gps_odom_points:
        oxs = [p[0] for _, p in gps_odom_points]
        oys = [p[1] for _, p in gps_odom_points]
        ax.plot(oxs, oys, "-", color="tab:cyan", linewidth=1, marker=".",
                markersize=3, alpha=0.8, label="navsat_transform (odometry/gps)")
    if odom_points:
        dxs = [p[0] for _, p in odom_points]
        dys = [p[1] for _, p in odom_points]
        odom_label = ("odom (dead-reckoned approx from odom_vx/vy/vyaw)" if odom_dead_reckoned
                       else "odom (raw, pre-fusion)")
        ax.plot(dxs, dys, "-", color="tab:brown", linewidth=1, marker=".",
                markersize=3, alpha=0.8, label=odom_label)

    for i0, i1, dist in fused_jumps:
        ax.plot([xs[i0], xs[i1]], [ys[i0], ys[i1]], "--", color="tab:red", linewidth=1.5)
        ax.plot(xs[i1], ys[i1], "x", color="tab:red", markersize=10, markeredgewidth=2)
        ax.annotate(f"{dist:.1f} m", xy=(xs[i1], ys[i1]), color="tab:red", fontsize=8)

    ax.plot(xs[0], ys[0], "o", color="tab:green", markersize=10, label="start")
    ax.plot(xs[-1], ys[-1], "o", color="tab:purple", markersize=10, label="end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(args.csv_path)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linewidth=0.3)
    ax.legend()

    if cov_idx:
        ax_cov.semilogy(cov_idx, cov_xx, "-", color="tab:blue", linewidth=1, label="cov_xx")
        ax_cov.semilogy(cov_idx, cov_yy, "-", color="tab:orange", linewidth=1, label="cov_yy")
        for i0, i1, dist in fused_jumps:
            ax_cov.axvline(i1, color="tab:red", linestyle="--", linewidth=1,
                            label="fused jump" if (i0, i1, dist) == fused_jumps[0] else None)
        ax_cov.set_xlabel("row idx")
        ax_cov.set_ylabel("position covariance [m^2] (log scale)")
        ax_cov.set_title("ekf_global reported x/y covariance")
        ax_cov.grid(True, linewidth=0.3)
        ax_cov.legend()
    else:
        ax_cov.text(0.5, 0.5, "no cov_xx/cov_yy in this CSV\n(re-record with updated "
                    "waypoint_recorder_node)", ha="center", va="center")
        ax_cov.set_axis_off()

    idxs = list(range(len(rows)))
    if any(v is not None for v in imu_yaws):
        ax_yaw.plot(idxs, [v if v is None else math.degrees(v) for v in fused_yaws],
                    "-", color="tab:blue", linewidth=1, label="fused yaw (odometry/filtered_map)")
        ax_yaw.plot(idxs, [v if v is None else math.degrees(v) for v in imu_yaws],
                    "-", color="tab:red", linewidth=1, alpha=0.8, label="raw imu yaw (imu, pre-fusion)")
        for i0, i1, dist in fused_jumps:
            ax_yaw.axvline(i1, color="tab:red", linestyle="--", linewidth=1,
                            label="fused jump" if (i0, i1, dist) == fused_jumps[0] else None)
        ax_yaw.set_xlabel("row idx")
        ax_yaw.set_ylabel("yaw [deg]")
        ax_yaw.set_title("fused yaw vs. raw imu orientation yaw")
        ax_yaw.grid(True, linewidth=0.3)
        ax_yaw.legend()
    else:
        ax_yaw.text(0.5, 0.5, "no imu_yaw in this CSV\n(re-record with updated "
                    "waypoint_recorder_node)", ha="center", va="center")
        ax_yaw.set_axis_off()

    if any(v is not None for v in imu_wz):
        ax_imu.plot(idxs, imu_wz, "-", color="tab:purple", linewidth=1, label="imu_wz [rad/s]")
        ax_imu.plot(idxs, imu_ax, "-", color="tab:green", linewidth=1, alpha=0.7, label="imu_ax [m/s^2]")
        ax_imu.plot(idxs, imu_ay, "-", color="tab:olive", linewidth=1, alpha=0.7, label="imu_ay [m/s^2]")
        for i0, i1, dist in fused_jumps:
            ax_imu.axvline(i1, color="tab:red", linestyle="--", linewidth=1,
                            label="fused jump" if (i0, i1, dist) == fused_jumps[0] else None)
        ax_imu.set_xlabel("row idx")
        ax_imu.set_ylabel("imu reading")
        ax_imu.set_title("raw imu angular velocity / linear acceleration")
        ax_imu.grid(True, linewidth=0.3)
        ax_imu.legend()
    else:
        ax_imu.text(0.5, 0.5, "no imu data in this CSV", ha="center", va="center")
        ax_imu.set_axis_off()

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
