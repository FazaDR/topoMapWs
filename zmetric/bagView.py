#!/usr/bin/env python3
"""Open3D viewer for /mapping/processed_scan (+ optional ground truth and /tf poses).

    source ~/topoMapWs/install/setup.bash
    source ~/topoMapWs/zmetric/.venv/bin/activate
    pip install open3d            # once
    python bag_view_o3d.py

Keys:  Right/Left = +-1 frame   Up/Down = +-10 frames
       B = toggle accumulated cloud   G = toggle GT   T = toggle /tf arrows   P = toggle pose arrows
"""
import bisect
import numpy as np
import open3d as o3d
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ============================ SETTINGS (edit here) ============================
BAG_DIR      = "/home/faza/topoMapWs/win_ideal_2"
STORAGE_ID   = "sqlite3"
TOPIC        = "/mapping/processed_scan"
TF_TOPIC     = "/tf"
TF_CHILD     = "base_link"
GT_PATH      = "/home/faza/topoMapWs/zmetric/gt_winding.npz"   # None = no ground truth
GT_MODULE    = "ground_truth"

BG_STRIDE    = 12          # keep every Nth point of every scan in the accumulated cloud
SCAN_STRIDE  = 2           # keep every Nth point for the current scan
Z_MIN_SHOW   = 0.10        # hide the floor (set -1 to show it)
Z_MAX_SHOW   = 3.0
BG_COLOR_BY  = "frame"     # "frame" | "height"
ERR_CLIM     = 0.5         # distance-to-GT color scale [m]
MAX_FRAMES   = None

POSE_EVERY   = 5           # draw a recorded-pose arrow every N frames
ARROW_LEN    = 0.8         # length of pose arrows [m]
SHOW_TF      = True        # orange arrows: /tf base_link pose at each scan stamp
SHOW_GT_SURFACE = False    # also draw GT surface samples as points
POINT_SIZE   = 2.0
START_FRAME  = 0
# ==============================================================================

TS = lambda s: s.sec + s.nanosec * 1e-9


def quat_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


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
    tf_t = get_message(types[TF_TOPIC]) if (SHOW_TF and TF_TOPIC in types) else None

    frames, tf = [], []            # tf: (t, pos, R)
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == TOPIC:
            if MAX_FRAMES and len(frames) >= MAX_FRAMES:
                continue
            m = deserialize_message(data, scan_t)
            p, q = m.robot_pose.pose.position, m.robot_pose.pose.orientation
            frames.append(dict(id=m.source_frame_id, t=TS(m.header.stamp),
                               pos=np.array([p.x, p.y, p.z]),
                               R=quat_R(q.x, q.y, q.z, q.w),
                               xyz=xyz_from_cloud(m.cloud)))
        elif tf_t is not None and topic == TF_TOPIC:
            for tr in deserialize_message(data, tf_t).transforms:
                if tr.child_frame_id == TF_CHILD:
                    v, q = tr.transform.translation, tr.transform.rotation
                    tf.append((TS(tr.header.stamp), np.array([v.x, v.y, v.z]),
                               quat_R(q.x, q.y, q.z, q.w)))
    if not frames:
        raise SystemExit("No scans read.")
    return frames, tf


def ramp(v, vmin=0.0, vmax=1.0):
    """0 -> blue, 0.5 -> green, 1 -> red."""
    t = np.clip((np.asarray(v) - vmin) / max(vmax - vmin, 1e-9), 0, 1)
    return np.column_stack([np.clip(2 * t - 1, 0, 1),
                            1 - np.abs(2 * t - 1),
                            np.clip(1 - 2 * t, 0, 1)])


def arrows_lineset(poses, length, color):
    """poses: list of (pos, R). Forward = +X of R. Shaft plus two head strokes."""
    pts, lines = [], []
    for pos, R in poses:
        f, l = R[:, 0], R[:, 1]
        tip = pos + f * length
        b = tip - f * length * 0.25
        i = len(pts)
        pts += [pos, tip, b + l * length * 0.1, b - l * length * 0.1]
        lines += [[i, i + 1], [i + 1, i + 2], [i + 1, i + 3]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.array(pts))
    ls.lines = o3d.utility.Vector2iVector(np.array(lines))
    ls.paint_uniform_color(color)
    return ls


def gt_wireframe(gt):
    pts, lines = [], []
    edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for c, R, h in zip(gt.centers, gt.Rs, gt.halves):
        i = len(pts)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(c + R @ (np.array([sx, sy, sz]) * h))
        lines += [[i + a, i + b] for a, b in edges]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.array(pts))
    ls.lines = o3d.utility.Vector2iVector(np.array(lines))
    ls.paint_uniform_color([0.55, 0.55, 0.55])
    return ls


def align_z_to(d):
    """Rotation taking +Z onto unit vector d."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, d)
    c = float(np.dot(z, d))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1 / (1 + c))


def main():
    frames, tf = load_bag()
    n = len(frames)
    t0 = frames[0]["t"]
    ids = np.array([f["id"] for f in frames])
    print(f"{n} frames, {frames[-1]['t'] - t0:.1f} s, "
          f"source_frame_id gaps: {int(np.sum(np.diff(ids) > 1))}, tf samples: {len(tf)}")

    gt = None
    if GT_PATH:
        gt = __import__(GT_MODULE).GroundTruth.load(GT_PATH)

    # ---------- accumulated cloud ----------
    P, C = [], []
    for i, f in enumerate(frames):
        p = f["xyz"][::BG_STRIDE]
        p = p[(p[:, 2] > Z_MIN_SHOW) & (p[:, 2] < Z_MAX_SHOW)]
        P.append(p)
        C.append(np.full(len(p), i / max(n - 1, 1)) if BG_COLOR_BY == "frame"
                 else (p[:, 2] - Z_MIN_SHOW) / (Z_MAX_SHOW - Z_MIN_SHOW))
    P, C = np.vstack(P), np.concatenate(C)
    bg = o3d.geometry.PointCloud()
    bg.points = o3d.utility.Vector3dVector(P)
    bgcol = ramp(C) * 0.6 + 0.2
    bg.colors = o3d.utility.Vector3dVector(bgcol)

    # ---------- recorded pose arrows + trajectory ----------
    sel = list(range(0, n, POSE_EVERY))
    pose_arrows = arrows_lineset([(frames[i]["pos"], frames[i]["R"]) for i in sel],
                                 ARROW_LEN, [0.1, 0.8, 0.2])
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(np.array([f["pos"] for f in frames]))
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(n - 1)])
    traj.paint_uniform_color([1, 1, 1])

    tf_arrows = None
    if tf:
        tt = np.array([x[0] for x in tf])
        poses = []
        for i in sel:
            j = bisect.bisect_right(tt, frames[i]["t"]) - 1
            if j >= 0:
                poses.append((tf[j][1], tf[j][2]))
        tf_arrows = arrows_lineset(poses, ARROW_LEN * 1.15, [1.0, 0.55, 0.0])

    gt_ls = gt_wireframe(gt) if gt is not None else None
    gt_pts = None
    if gt is not None and SHOW_GT_SURFACE:
        gt_pts = o3d.geometry.PointCloud()
        s = gt.surface[gt.surface[:, 2] > Z_MIN_SHOW]
        gt_pts.points = o3d.utility.Vector3dVector(s)
        gt_pts.paint_uniform_color([0.4, 0.4, 0.4])

    # ---------- current frame ----------
    cur = o3d.geometry.PointCloud()
    base_arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.04, cone_radius=0.09,
        cylinder_height=ARROW_LEN * 0.7, cone_height=ARROW_LEN * 0.3)
    base_v = np.asarray(base_arrow.vertices).copy()
    cur_arrow = o3d.geometry.TriangleMesh(base_arrow)
    cur_arrow.paint_uniform_color([1.0, 0.1, 0.1])
    cur_arrow.compute_vertex_normals()

    state = {"i": START_FRAME}

    def update(i):
        i = int(np.clip(i, 0, n - 1))
        state["i"] = i
        f = frames[i]
        p = f["xyz"][::SCAN_STRIDE]
        p = p[(p[:, 2] > Z_MIN_SHOW) & (p[:, 2] < Z_MAX_SHOW)]
        cur.points = o3d.utility.Vector3dVector(p)
        msg = ""
        if gt is not None:
            d = gt.dist_to_surface(p)
            cur.colors = o3d.utility.Vector3dVector(ramp(d, 0, ERR_CLIM))
            w = d[p[:, 2] > 0.1]
            if len(w):
                msg = f"  wall->GT p50={np.median(w):.3f} p90={np.percentile(w, 90):.3f} m"
        else:
            cur.paint_uniform_color([1, 0.2, 0.2])
        T = align_z_to(f["R"][:, 0])               # arrow +Z -> heading (+X of pose)
        cur_arrow.vertices = o3d.utility.Vector3dVector(base_v @ T.T + f["pos"])
        cur_arrow.compute_vertex_normals()
        print(f"frame {i}/{n - 1} id={f['id']} t={f['t'] - t0:.2f}s pts={len(f['xyz'])}{msg}")

    update(START_FRAME)

    # ---------- viewer ----------
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("bag view", 1500, 900)
    opt = vis.get_render_option()
    opt.point_size = POINT_SIZE
    opt.background_color = np.array([0.08, 0.08, 0.1])

    shown = {}
    def add(name, g):
        if g is not None:
            vis.add_geometry(g, reset_bounding_box=(name == "bg"))
            shown[name] = [g, True]
    add("bg", bg); add("traj", traj); add("pose", pose_arrows); add("tf", tf_arrows)
    add("gt", gt_ls); add("gtpts", gt_pts); add("cur", cur); add("arrow", cur_arrow)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

    def refresh():
        vis.update_geometry(cur)
        vis.update_geometry(cur_arrow)
        vis.update_renderer()

    def step(d):
        def cb(_):
            update(state["i"] + d)
            refresh()
            return False
        return cb

    def toggle(name):
        def cb(_):
            if name in shown:
                g, on = shown[name]
                (vis.remove_geometry if on else
                 lambda x: vis.add_geometry(x, reset_bounding_box=False))(g)
                shown[name][1] = not on
            return False
        return cb

    for key, d in ((262, 1), (263, -1), (265, 10), (264, -10)):   # right/left/up/down
        vis.register_key_callback(key, step(d))
    vis.register_key_callback(ord("B"), toggle("bg"))
    vis.register_key_callback(ord("G"), toggle("gt"))
    vis.register_key_callback(ord("T"), toggle("tf"))
    vis.register_key_callback(ord("P"), toggle("pose"))

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()