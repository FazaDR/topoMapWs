#!/usr/bin/env python3
"""
Compare mapping runs: process time vs node count, and map quality against ground truth.

  python eval_runs.py      (paths are set in SETTINGS below)
  
Each RUN is a folder written by mapping_node:
  metrics.csv            per-frame total_ms and node counts   -> time plot
  map_layer0_nodes.csv   saved map (leaf layer)               -> quality summary
  params.txt             used for the legend label (algorithm, free-area)
ground_truth.py must sit next to this script.

Reference for the quality summary = the OBSERVABLE ground-truth surface: exposed faces that
face free space reachable by the robot (faces hidden inside other boxes, the floor underside,
wall tops and the outside of the arena are skipped, otherwise coverage could never reach 100%).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

try:
    from ground_truth import GroundTruth
except ImportError:
    sys.exit("ground_truth.py must be in the same folder as this script")


# ----------------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------------
def read_params(run):
    p = {}
    f = run / "params.txt"
    if f.exists():
        for line in f.read_text().splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                p[k.strip()] = v.strip()
    return p


def make_label(run, params):
    alg = params.get("algorithm", run.name).upper()
    free = params.get("free_area_enabled") == "1"
    return alg + (" + free-area" if free else "")


def load_metrics(run, nodes_col):
    f = run / "metrics.csv"
    if not f.exists():
        sys.exit(f"missing {f}")
    m = np.atleast_1d(np.genfromtxt(f, delimiter=",", names=True))
    if m.size == 0 or nodes_col not in (m.dtype.names or ()):
        sys.exit(f"{f}: no rows, or no column '{nodes_col}'")
    return m[nodes_col].astype(float), m["total_ms"].astype(float) / 1000.0


def load_nodes(run, layer):
    f = run / f"map_layer{layer}_nodes.csv"
    if not f.exists():
        sys.exit(f"missing {f} (map is only saved with replay.save_map / "
                 f"metrics.save_map_on_shutdown)")
    d = np.atleast_1d(np.genfromtxt(f, delimiter=",", names=True))
    if d.size == 0 or "x" not in (d.dtype.names or ()):
        sys.exit(f"{f}: no nodes")
    return np.column_stack([d["x"], d["y"], d["z"]]).astype(np.float64)


# ----------------------------------------------------------------------------
# Ground-truth reference surface
# ----------------------------------------------------------------------------
def chunked(fn, pts, n=200_000):
    if len(pts) == 0:
        return np.zeros(0)
    return np.concatenate([fn(pts[i:i + n]) for i in range(0, len(pts), n)])


def observable_surface(gt, step, start_xy, z_max):
    """Exposed GT surface samples that face robot-reachable free space.

    Returns (points, normals, reachable_area_m2).
    """
    # reachable free space: connected component of the (non-inflated) 2D grid
    labels, _ = ndimage.label(~gt.grid)
    if start_xy is None:
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        lab = int(sizes.argmax())               # largest free region = the arena interior
    else:
        r, c = gt.world_to_cell(np.array([start_xy], float))
        r, c = int(r[0]), int(c[0])
        if not (0 <= r < labels.shape[0] and 0 <= c < labels.shape[1]) or labels[r, c] == 0:
            sys.exit(f"--start {start_xy} is not in free space of the ground-truth grid")
        lab = int(labels[r, c])
    reach = labels == lab
    area = float(reach.sum() * gt.res ** 2)

    # sample every box face, keeping the outward normal
    pts, nrm = [], []
    for ctr, R, h in zip(gt.centers, gt.Rs, gt.halves):
        for ax in range(3):
            o = [i for i in range(3) if i != ax]
            u = np.linspace(-h[o[0]], h[o[0]], max(int(2 * h[o[0]] / step) + 1, 2))
            v = np.linspace(-h[o[1]], h[o[1]], max(int(2 * h[o[1]] / step) + 1, 2))
            U, V = np.meshgrid(u, v)
            for s in (-1.0, 1.0):
                L = np.zeros((U.size, 3))
                L[:, ax] = s * h[ax]
                L[:, o[0]] = U.ravel()
                L[:, o[1]] = V.ravel()
                pts.append(L @ R.T + ctr)
                n = np.zeros(3)
                n[ax] = s
                nrm.append(np.tile(R @ n, (U.size, 1)))
    P, N = np.vstack(pts), np.vstack(nrm)

    keep = P[:, 2] >= -1e-3                      # drops floor underside / below-floor sides
    if z_max is not None:
        keep &= P[:, 2] <= z_max
    P, N = P[keep], N[keep]

    exposed = ~chunked(gt.inside_any, P + 0.01 * N)   # face not buried in another box
    P, N = P[exposed], N[exposed]

    Q = P + 1.5 * gt.res * N                      # probe just outside the face
    rows, cols = gt.world_to_cell(Q[:, :2])
    inb = (rows >= 0) & (rows < reach.shape[0]) & (cols >= 0) & (cols < reach.shape[1])
    ok = np.zeros(len(P), bool)
    ok[inb] = reach[rows[inb], cols[inb]]
    return P[ok], N[ok], area


# ----------------------------------------------------------------------------
# Time fits
# ----------------------------------------------------------------------------
def fit_curve(n, t, kind):
    ok = n > 0
    n, t = n[ok], t[ok]
    if kind == "log":
        a, b = np.polyfit(np.log(n), t, 1)
        return (lambda x: a * np.log(x) + b), f"time = {a:.4f} * ln(nodes) + {b:.4f}"
    a, b = np.polyfit(n, t, 1)
    return (lambda x: a * x + b), f"time = {a:.3e} * nodes + {b:.4f}"


# ----------------------------------------------------------------------------
# Quality summary
# ----------------------------------------------------------------------------
def evaluate(name, S, nodes, gt, thresholds, node_tol):
    tree = cKDTree(nodes)
    d_ref = tree.query(S)[0]                              # surface point -> nearest node
    d_node = chunked(gt.dist_to_surface, nodes)           # node -> nearest GT surface
    avg_nn = tree.query(nodes, k=2)[0][:, 1].mean() if len(nodes) > 1 else float("nan")

    bar = "=" * 70
    print(f"\n{bar}\n{name} EVALUATION\n{bar}")
    print(f"Reference points : {len(S)}  (observable ground-truth surface)")
    print(f"Map nodes        : {len(nodes)}\n")
    print("Reference -> nearest node")
    print(f"Mean distance    : {d_ref.mean():.6f} m")
    print(f"RMSE             : {np.sqrt(np.mean(d_ref ** 2)):.6f} m")
    print(f"Maximum distance : {d_ref.max():.6f} m")
    print(f"Avg node distance: {avg_nn:.6f} m\n")
    for t in thresholds:
        print(f"Coverage <= {t:.2f} m : {100.0 * np.mean(d_ref <= t):.2f}%")
    print("\nNode -> ground-truth surface")
    print(f"Mean distance    : {d_node.mean():.6f} m")
    print(f"RMSE             : {np.sqrt(np.mean(d_node ** 2)):.6f} m")
    print(f"Maximum distance : {d_node.max():.6f} m")
    print(f"Within {node_tol:.2f} m     : {100.0 * np.mean(d_node <= node_tol):.2f}%")
    print(bar)


# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# SETTINGS (edit here)
# ----------------------------------------------------------------------------
GT_PATH = "/home/faza/topoMapWs/zmetric/gt_winding.npz"
RUNS = [
    "/home/faza/topoMapWs/zmapping_out/atc_1000_20261119_051157",
    "/home/faza/topoMapWs/zmapping_out/mlatc_1000_20261119_051102",
]
LAYER = 0                       # map_layer0_*.csv = leaf layer ("layer 1" = metrics column layer1_nodes)
NODES_COL = "layer1_nodes"      # x axis of the time plot
THRESHOLDS = [0.15, 0.2, 0.25]
NODE_TOL = 0.10
STEP = 0.1                      # reference surface spacing [m]
START = None                    # (x, y) in robot free space, or None = largest free region
Z_MAX = None
OUT = "time_vs_nodes.png"


def main():
    from types import SimpleNamespace
    a = SimpleNamespace(gt=GT_PATH, runs=RUNS, layer=LAYER, nodes_col=NODES_COL, fit="auto",
                        thresholds=THRESHOLDS, node_tol=NODE_TOL, step=STEP, start=START,
                        z_max=Z_MAX, out=OUT, no_show=False)
    if len(sys.argv) > 1 and sys.argv[1] == "--no-show":   # only for headless testing
        a.no_show = True
        a.gt, a.runs, a.out = sys.argv[2], sys.argv[3:5], sys.argv[5]

    import matplotlib.pyplot as plt

    runs = [Path(r).expanduser() for r in a.runs]
    params = [read_params(r) for r in runs]
    labels = [make_label(r, p) for r, p in zip(runs, params)]
    if len(set(labels)) < len(labels):
        labels = [f"{lab} [{r.name}]" for lab, r in zip(labels, runs)]

    # ---------------- time vs node count ----------------
    plt.figure(figsize=(10, 6))
    print("PROCESS TIME")
    for run, p, lab in zip(runs, params, labels):
        n, t = load_metrics(run, a.nodes_col)
        kind = a.fit if a.fit != "auto" else ("log" if p.get("algorithm") == "mlatc" else "linear")
        (line,) = plt.plot(n, t*1000, marker="o", markersize=3, linewidth=1.5, label=lab)
        f, eq = fit_curve(n, t, kind)
        xs = np.sort(n[n > 0])
        plt.plot(xs, f(xs)*1000, linestyle="--", linewidth=2, color=line.get_color(),
                 label=f"{lab} {kind} fit")
        print(f"  {lab}: frames {len(t)}, mean {t.mean():.4f} s, "
              f"p95 {np.percentile(t, 95):.4f} s, max {t.max():.4f} s")
        print(f"    {kind} estimate: {eq}")
    plt.xlabel("Node Count")
    plt.ylabel("Process Time (ms)")
    plt.title("Node Count vs Process Time")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(a.out, dpi=150)
    print(f"  plot saved: {a.out}")

    # ---------------- map quality ----------------
    gt = GroundTruth.load(a.gt)
    S, _, area = observable_surface(gt, a.step, a.start, a.z_max)
    print(f"\nReference: {len(S)} observable surface points "
          f"(reachable free space {area:.0f} m^2)")
    for run, lab in zip(runs, labels):
        evaluate(lab, S, load_nodes(run, a.layer), gt, a.thresholds, a.node_tol)

    if not a.no_show:
        plt.show()


if __name__ == "__main__":
    main()