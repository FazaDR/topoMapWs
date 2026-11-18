import bisect, sys
import numpy as np
from scipy.spatial import cKDTree
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/faza/topoMapWs/win_ideal_2"
NODES = sys.argv[1]            # ..._layer0_nodes.csv

ts = lambda s: s.sec + s.nanosec * 1e-9
yaw = lambda q: np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}

tf_t, tf_xy, tf_yaw, scans, pts = [], [], [], [], []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == "/tf":
        for tr in deserialize_message(data, get_message(types[topic])).transforms:
            if tr.child_frame_id == "base_link":
                tf_t.append(ts(tr.header.stamp))
                tf_xy.append([tr.transform.translation.x, tr.transform.translation.y])
                tf_yaw.append(yaw(tr.transform.rotation))
    elif topic == "/mapping/processed_scan":
        m = deserialize_message(data, get_message(types[topic]))
        p, q = m.robot_pose.pose.position, m.robot_pose.pose.orientation
        scans.append((ts(m.header.stamp), p.x, p.y, yaw(q)))
        a = np.frombuffer(bytes(m.cloud.data), np.float32).reshape(-1, m.cloud.point_step // 4)
        xyz = a[::7, :3]
        pts.append(xyz[np.isfinite(xyz).all(axis=1)])                  # subsample

tf_t, tf_xy, tf_yaw = map(np.array, (tf_t, tf_xy, tf_yaw))
dpos, dyaw = [], []
for t, x, y, yw in scans:
    i = bisect.bisect_right(tf_t, t) - 1
    if i < 0: continue
    dpos.append(np.hypot(x - tf_xy[i, 0], y - tf_xy[i, 1]))
    d = yw - tf_yaw[i]; dyaw.append(abs(np.arctan2(np.sin(d), np.cos(d))))
dpos, dyaw = np.array(dpos), np.degrees(dyaw)
print("pose used vs pose at scan stamp [m]  p50/p90/p99/max:",
      np.percentile(dpos, [50, 90, 99]).round(3), dpos.max().round(3))
print("yaw error [deg]                       p50/p90/p99/max:",
      np.percentile(dyaw, [50, 90, 99]).round(2), dyaw.max().round(2))

# nodes vs. input points
import pandas as pd
n = pd.read_csv(NODES)
tree = cKDTree(np.vstack(pts))
d, _ = tree.query(n[["x", "y", "z"]].to_numpy())
gray = (n[["nx", "ny", "nz"]].abs().sum(axis=1) < 1e-8).to_numpy()
for name, mask in (("gray", gray), ("colored", ~gray)):
    print(f"{name:8s} n={mask.sum():5d}  dist to nearest scan point p50/p90/max:",
          np.percentile(d[mask], [50, 90]).round(3), d[mask].max().round(3))
print("nodes farther than 0.3 m from any point:", (d > 0.3).sum(), "of", len(d))