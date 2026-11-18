#!/usr/bin/env python3
"""Check /mapping/processed_scan in a bag against the ground truth made from the world file.

    source ~/topoMapWs/install/setup.bash
    source ~/topoMapWs/zmetric/.venv/bin/activate
    python bag_check_gt.py

Everything is hardcoded in SETTINGS below. Put this file next to ground_truth.py
(the module that holds GroundTruth). It prints:

  1. pose sanity     recorded sensor poses vs the GT occupancy grid (are they in free space?)
  2. scan -> GT      distance of every wall point to the nearest GT surface, for all frames and
                     split by motion: stationary / driving / turning
  3. time-lag sweep  re-places every scan with the /tf pose at (stamp + lag) and reports which lag
                     fits the walls best. Best lag != 0 means scan stamps and poses are offset.

It also saves gt_check.csv (per frame) and gt_check.png next to this script.
"""
import csv
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
    
# ============================ SETTINGS (edit here) ============================
BAG_DIR       = "/home/faza/topoMapWs/win_ideal_2"
STORAGE_ID    = "sqlite3"
TOPIC         = "/mapping/processed_scan"
TF_TOPIC      = "/tf"
TF_CHILD      = "base_link"
GT_PATH       = "/home/faza/topoMapWs/zmetric/gt_winding.npz"
GT_MODULE     = "ground_truth"          # module with class GroundTruth (change if your file is gt.py)

Z_MIN         = 0.10                    # wall slice: hides the floor
Z_MAX         = 2.40                    # walls are 2.5 m tall; hides the top edges
POINTS_PER_FRAME = 4000                 # random subsample per scan (keeps it fast)
LAG_POINTS    = 1000                    # points per scan used in the lag sweep
MAX_FRAMES    = None                    # None = all
T_MIN_S       = None                    # only frames after this many s from the first scan (None = all)
T_MAX_S       = None                    # only frames before this many s (use for divider_room, see below)

BAD_DIST      = 0.10                    # a point farther than this from any GT surface counts as "bad" [m]
STILL_SPEED   = 0.05                    # [m/s]   below this and below STILL_YAWRATE = stationary
STILL_YAWRATE = 0.05                    # [rad/s]
TURN_YAWRATE  = 0.20                    # [rad/s] above this = turning
LAG_RANGE     = np.arange(-0.10, 0.1001, 0.01)   # [s]

OUT_DIR       = os.path.dirname(os.path.abspath(__file__))
# ==============================================================================
# divider_room note: the orange wall is removed at 30 s. Either use the matching GT file for the
# part of the run you check (gt_divider_full.npz before 30 s, gt_divider_removed.npz after), by
# setting T_MIN_S / T_MAX_S, or the points on that wall will count as errors.

TS = lambda s: s.sec + s.nanosec * 1e-9


def xyz_from_cloud(c):
    off = {f.name: f.offset for f in c.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": c.point_step})
    a = np.frombuffer(bytes(c.data), dtype=dt)
    p = np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]


def load_bag():
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG_DIR, storage_id=STORAGE_ID), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if TOPIC not in types:
        raise SystemExit(f"{TOPIC} not in bag. Topics: {list(types)}")
    scan_t = get_message(types[TOPIC])
    tf_t = get_message(types[TF_TOPIC]) if TF_TOPIC in types else None
    rng = np.random.default_rng(0)

    frames, tf = [], []                      # tf: (t, pos, quat xyzw)
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == TOPIC:
            if MAX_FRAMES and len(frames) >= MAX_FRAMES:
                continue
            m = deserialize_message(data, scan_t)
            p = xyz_from_cloud(m.cloud)
            p = p[(p[:, 2] > Z_MIN) & (p[:, 2] < Z_MAX)]
            if len(p) > POINTS_PER_FRAME:
                p = p[rng.choice(len(p), POINTS_PER_FRAME, replace=False)]
            pp, q = m.robot_pose.pose.position, m.robot_pose.pose.orientation
            frames.append(dict(id=m.source_frame_id, t=TS(m.header.stamp),
                               pos=np.array([pp.x, pp.y, pp.z]),
                               q=np.array([q.x, q.y, q.z, q.w]), xyz=p))
        elif tf_t is not None and topic == TF_TOPIC:
            for tr in deserialize_message(data, tf_t).transforms:
                if tr.child_frame_id == TF_CHILD:
                    v, q = tr.transform.translation, tr.transform.rotation
                    tf.append((TS(tr.header.stamp), np.array([v.x, v.y, v.z]),
                               np.array([q.x, q.y, q.z, q.w])))
    if not frames:
        raise SystemExit("No scans read.")
    return frames, tf


def stat_line(name, d, nframes):
    if len(d) == 0:
        return f"  {name:<11} frames={nframes:4d}  (no points)"
    return (f"  {name:<11} frames={nframes:4d} pts={len(d):8d}  "
            f"mean={d.mean() * 100:5.1f}  p50={np.median(d) * 100:5.1f}  "
            f"p90={np.percentile(d, 90) * 100:5.1f}  p99={np.percentile(d, 99) * 100:5.1f} cm  "
            f"bad(>{BAD_DIST * 100:.0f}cm)={100 * (d > BAD_DIST).mean():5.1f}%")


def main():
    sys.path.insert(0, os.path.dirname(GT_PATH))
    gt = __import__(GT_MODULE).GroundTruth.load(GT_PATH)

    frames, tf = load_bag()
    t_first = frames[0]["t"]
    frames = [f for f in frames
              if (T_MIN_S is None or f["t"] - t_first >= T_MIN_S)
              and (T_MAX_S is None or f["t"] - t_first <= T_MAX_S)]
    n = len(frames)
    if n < 3:
        raise SystemExit(f"Only {n} frames after filtering, need at least 3.")

    t = np.array([f["t"] for f in frames])
    P = np.array([f["pos"] for f in frames])                    # sensor position (map frame)
    Rr = Rotation.from_quat(np.array([f["q"] for f in frames]))
    Rmat = Rr.as_matrix()
    yaw = np.unwrap(Rr.as_euler("ZYX")[:, 0])
    speed = np.linalg.norm(np.gradient(P, t, axis=0), axis=1)
    yawrate = np.gradient(yaw, t)

    print(f"{n} frames, {t[-1] - t[0]:.1f} s, tf samples: {len(tf)}, GT boxes: {len(gt.names)}")

    # ---------------- 1. pose sanity ----------------
    print("\n[1] pose sanity (recorded sensor pose vs GT occupancy grid)")
    free = gt.is_free(P[:, :2], inflated=True)
    print(f"  first pose ({P[0, 0]:.2f}, {P[0, 1]:.2f}, {P[0, 2]:.2f})   "
          f"poses in blocked/inflated cells: {int((~free).sum())} of {n}")
    if (~free).any():
        i = int(np.argmax(~free))
        print(f"  first blocked pose: frame {i} at ({P[i, 0]:.2f}, {P[i, 1]:.2f})  "
              f"-> robot path touches walls, or GT and sim frames are offset")

    # ---------------- 2. scan -> GT ----------------
    print("\n[2] scan -> GT distance (wall points, z in "
          f"[{Z_MIN}, {Z_MAX}] m)")
    dists = [gt.dist_to_surface(f["xyz"]) for f in frames]
    p50 = np.array([np.median(d) if len(d) else np.nan for d in dists])
    p90 = np.array([np.percentile(d, 90) if len(d) else np.nan for d in dists])
    bad = np.array([(d > BAD_DIST).mean() if len(d) else np.nan for d in dists])

    still = (speed < STILL_SPEED) & (np.abs(yawrate) < STILL_YAWRATE)
    turning = np.abs(yawrate) > TURN_YAWRATE
    driving = ~still & ~turning
    for name, mask in (("all", np.ones(n, bool)), ("stationary", still),
                       ("driving", driving), ("turning", turning)):
        d = np.concatenate([dists[i] for i in np.flatnonzero(mask)]) if mask.any() else np.array([])
        print(stat_line(name, d, int(mask.sum())))

    ok = ~np.isnan(p50)
    if ok.sum() > 3:
        print(f"  corr(frame p50, speed)    = {np.corrcoef(p50[ok], speed[ok])[0, 1]:+.2f}")
        print(f"  corr(frame p50, |yawrate|) = {np.corrcoef(p50[ok], np.abs(yawrate[ok]))[0, 1]:+.2f}")
        print("  (strong positive = error grows with motion -> timing/sync; ~0 = error is constant "
              "-> transform, offsets or lidar resolution)")

    # ---------------- 3. time-lag sweep ----------------
    print("\n[3] time-lag sweep (scan re-placed with pose at stamp + lag)")
    if tf:
        order = np.argsort([x[0] for x in tf])
        tt = np.array([tf[i][0] for i in order])
        tp = np.array([tf[i][1] for i in order])
        tq = np.array([tf[i][2] for i in order])
        tt, uniq = np.unique(tt, return_index=True)
        tp, tq = tp[uniq], tq[uniq]
        src = f"{TF_TOPIC} ({TF_CHILD}) at about {1.0 / max(np.median(np.diff(tt)), 1e-9):.0f} Hz"
    else:
        tt, tp, tq = t, P, np.array([f["q"] for f in frames])
        src = "scan poses only (no /tf in bag; coarse, only a rough test)"
    slerp = Slerp(tt, Rotation.from_quat(tq))
    print(f"  pose source: {src}")

    def base_at(times):
        c = np.clip(times, tt[0], tt[-1])
        pos = np.column_stack([np.interp(c, tt, tp[:, k]) for k in range(3)])
        return pos, slerp(c).as_matrix()

    # sensor offset in the body frame (z offset for ddmr, zero for idealized_lidar)
    pos_b, R_b = base_at(t)
    off_body = np.median(np.einsum("nji,nj->ni", R_b, P - pos_b), axis=0) if tf else np.zeros(3)
    print(f"  sensor offset in body frame (estimated): {np.round(off_body, 3)}")

    local = [(f["xyz"][:LAG_POINTS] - P[i]) @ Rmat[i] for i, f in enumerate(frames)]  # sensor frame
    rows = []
    for lag in LAG_RANGE:
        pos_l, R_l = base_at(t + lag)
        sens = pos_l + np.einsum("nij,j->ni", R_l, off_body)
        d = np.concatenate([gt.dist_to_surface(local[i] @ R_l[i].T + sens[i]) for i in range(n)])
        rows.append((lag, np.median(d), d.mean(), np.percentile(d, 90)))
    rows = np.array(rows)
    best = int(np.argmin(rows[:, 2]))
    print("  lag [s]    p50 [cm]  mean [cm]  p90 [cm]")
    for k, (lag, m50, mean, q90) in enumerate(rows):
        mark = "  <- best" if k == best else ""
        print(f"  {lag:+.3f}   {m50 * 100:8.2f}  {mean * 100:9.2f}  {q90 * 100:8.2f}{mark}")
    zero = int(np.argmin(np.abs(rows[:, 0])))
    print(f"  best lag {rows[best, 0]:+.3f} s: scans fit the walls best when placed with the pose "
          f"from {abs(rows[best, 0]) * 1000:.0f} ms {'AFTER' if rows[best, 0] > 0 else 'BEFORE'} "
          f"the stamp; mean error {rows[best, 2] * 100:.2f} cm vs {rows[zero, 2] * 100:.2f} cm at lag 0")

    # ---------------- outputs ----------------
    csv_path = os.path.join(OUT_DIR, "gt_check.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame_id", "t_s", "speed", "yawrate", "pts", "p50_m", "p90_m", "bad_frac"])
        for i, f in enumerate(frames):
            w.writerow([f["id"], f"{f['t'] - t_first:.3f}", f"{speed[i]:.3f}", f"{yawrate[i]:.3f}",
                        len(f["xyz"]), f"{p50[i]:.4f}", f"{p90[i]:.4f}", f"{bad[i]:.4f}"])
    print(f"\nsaved {csv_path}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(18, 5))
        tr = t - t_first
        ax[0].plot(tr, p50 * 100, label="p50 [cm]")
        ax[0].plot(tr, p90 * 100, label="p90 [cm]")
        ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("scan -> GT [cm]"); ax[0].legend(loc="upper left")
        a2 = ax[0].twinx(); a2.plot(tr, speed, "g", alpha=0.35); a2.set_ylabel("speed [m/s] (green)")
        ax[0].set_title("error per frame")
        ax[1].plot(rows[:, 0], rows[:, 2] * 100, "o-", label="mean")
        ax[1].plot(rows[:, 0], rows[:, 1] * 100, "s-", label="p50")
        ax[1].axvline(0, color="k", lw=0.5); ax[1].set_xlabel("lag [s]"); ax[1].set_ylabel("[cm]")
        ax[1].legend(); ax[1].set_title("time-lag sweep")
        ext = [gt.origin[0], gt.origin[0] + gt.grid.shape[1] * gt.res,
               gt.origin[1], gt.origin[1] + gt.grid.shape[0] * gt.res]
        ax[2].imshow(gt.grid, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
        sc = ax[2].scatter(P[:, 0], P[:, 1], c=p50 * 100, s=8, cmap="viridis")
        fig.colorbar(sc, ax=ax[2], label="frame p50 [cm]")
        ax[2].set_aspect("equal"); ax[2].set_title("poses colored by error")
        png = os.path.join(OUT_DIR, "gt_check.png")
        fig.savefig(png, dpi=110, bbox_inches="tight")
        print(f"saved {png}")
    except Exception as e:                       # plotting is optional
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()