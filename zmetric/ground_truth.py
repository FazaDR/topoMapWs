#!/usr/bin/env python3
"""
Ground truth from an SDF world: oriented boxes -> surface points + 2D occupancy grid.

  python gt.py winding_room.world --out gt_winding.npz
  python gt.py divider_room.world --out gt_divider_full.npz
  python gt.py divider_room.world --exclude disappearing_wall --out gt_divider_removed.npz

Supports: <model>/<link>/<collision>/<geometry>/<box>, nested poses (model * link * collision),
<include> (resolved through --models-dir), roll/pitch/yaw rotation.
Ignores non-box geometry (a warning is printed).
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


# ----------------------------------------------------------------------------
# Pose helpers
# ----------------------------------------------------------------------------
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_pose(elem):
    """Return (R, t) from a child <pose> of elem (identity if missing)."""
    txt = elem.findtext("pose") if elem is not None else None
    if not txt:
        return np.eye(3), np.zeros(3)
    v = [float(x) for x in txt.split()]
    v += [0.0] * (6 - len(v))
    return rpy_to_R(v[3], v[4], v[5]), np.array(v[:3])


def compose(a, b):
    (R1, t1), (R2, t2) = a, b
    return R1 @ R2, R1 @ t2 + t1


# ----------------------------------------------------------------------------
# SDF parsing
# ----------------------------------------------------------------------------
def collect_boxes(model, parent, out, exclude, name_prefix=""):
    """Walk one <model>, appending (name, center, R, half_size) to out."""
    name = model.get("name", "")
    if name in exclude:
        return
    mp = compose(parent, parse_pose(model))

    for link in model.findall("link"):
        lp = compose(mp, parse_pose(link))
        for col in link.findall("collision"):
            cp = compose(lp, parse_pose(col))
            geom = col.find("geometry")
            box = geom.find("box/size") if geom is not None else None
            if box is not None:
                size = np.array([float(v) for v in box.text.split()])
                out.append((name, cp[1], cp[0], size / 2.0))
            elif geom is not None and len(geom):
                print(f"warning: model '{name}' has non-box collision "
                      f"<{geom[0].tag}>, ignored")

    for sub in model.findall("model"):          # nested models
        collect_boxes(sub, mp, out, exclude)


def load_boxes(world_path, exclude=(), models_dir=None):
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    world = world if world is not None else root
    out = []
    ident = (np.eye(3), np.zeros(3))

    for model in world.findall("model"):
        collect_boxes(model, ident, out, set(exclude))

    for inc in world.findall("include"):
        uri = (inc.findtext("uri") or "").replace("model://", "")
        name = inc.findtext("name") or uri
        if name in exclude or uri in exclude:
            continue
        if models_dir is None:
            print(f"warning: <include> '{uri}' skipped (pass --models-dir)")
            continue
        sdf = Path(models_dir) / uri / "model.sdf"
        if not sdf.exists():
            print(f"warning: {sdf} not found, '{uri}' skipped")
            continue
        inc_pose = parse_pose(inc)
        mroot = ET.parse(sdf).getroot()
        for model in mroot.iter("model"):
            if model.get("name") is not None:
                collect_boxes(model, inc_pose, out, set())
                break

    if not out:
        raise RuntimeError("No box collisions found in the world.")
    return out


# ----------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------
class GroundTruth:
    def __init__(self, names, centers, Rs, halves):
        self.names = list(names)
        self.centers = np.asarray(centers)      # (B,3)
        self.Rs = np.asarray(Rs)                # (B,3,3)
        self.halves = np.asarray(halves)        # (B,3)
        self.surface = None
        self.grid = self.grid_inflated = None
        self.origin = self.res = None

    # ---- distances ---------------------------------------------------------
    def dist_to_surface(self, p):
        """Unsigned distance (N,) from points p (N,3) to the nearest box surface."""
        p = np.asarray(p, dtype=np.float64)
        best = np.full(len(p), np.inf)
        for c, R, h in zip(self.centers, self.Rs, self.halves):
            local = (p - c) @ R                  # world -> box frame
            q = np.abs(local) - h
            outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
            inside = np.minimum(q.max(axis=1), 0.0)
            best = np.minimum(best, np.abs(outside + inside))
        return best

    def inside_any(self, p):
        p = np.asarray(p, dtype=np.float64)
        hit = np.zeros(len(p), bool)
        for c, R, h in zip(self.centers, self.Rs, self.halves):
            local = (p - c) @ R
            hit |= (np.abs(local) <= h).all(axis=1)
        return hit

    def segment_hits_box(self, a, b, shrink=0.0):
        """True if segment a->b passes through any box (slab test, per box)."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        for c, R, h in zip(self.centers, self.Rs, self.halves):
            la, lb = (a - c) @ R, (b - c) @ R
            d = lb - la
            t0, t1 = 0.0, 1.0
            ok = True
            hh = h - shrink
            for i in range(3):
                if abs(d[i]) < 1e-12:
                    if abs(la[i]) > hh[i]:
                        ok = False
                        break
                else:
                    u0, u1 = (-hh[i] - la[i]) / d[i], (hh[i] - la[i]) / d[i]
                    if u0 > u1:
                        u0, u1 = u1, u0
                    t0, t1 = max(t0, u0), min(t1, u1)
                    if t0 > t1:
                        ok = False
                        break
            if ok:
                return True
        return False

    # ---- surface points ------------------------------------------------------
    def sample_surface(self, step=0.05, min_z=-1e-3):
        out = []
        for c, R, h in zip(self.centers, self.Rs, self.halves):
            for ax in range(3):
                o = [i for i in range(3) if i != ax]
                u = np.linspace(-h[o[0]], h[o[0]], max(int(2 * h[o[0]] / step) + 1, 2))
                v = np.linspace(-h[o[1]], h[o[1]], max(int(2 * h[o[1]] / step) + 1, 2))
                U, V = np.meshgrid(u, v)
                for s in (-1.0, 1.0):
                    P = np.zeros((U.size, 3))
                    P[:, ax] = s * h[ax]
                    P[:, o[0]] = U.ravel()
                    P[:, o[1]] = V.ravel()
                    out.append(P @ R.T + c)
        pts = np.vstack(out)
        pts = pts[pts[:, 2] >= min_z]            # drops the floor's underside and sides
        self.surface = np.unique(np.round(pts, 4), axis=0)
        return self.surface

    # ---- occupancy grid ------------------------------------------------------
    def build_grid(self, res=0.05, z_min=0.05, z_max=2.0, robot_radius=0.3, margin=1.0):
        from scipy.ndimage import binary_dilation

        corners = []
        for c, R, h in zip(self.centers, self.Rs, self.halves):
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corners.append(c + R @ (np.array([sx, sy, sz]) * h))
        corners = np.array(corners)
        lo = corners[:, :2].min(axis=0) - margin
        hi = corners[:, :2].max(axis=0) + margin
        nx, ny = np.ceil((hi - lo) / res).astype(int)

        xs = lo[0] + (np.arange(nx) + 0.5) * res
        ys = lo[1] + (np.arange(ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)               # grid[row=y, col=x]
        cells = np.stack([X.ravel(), Y.ravel()], axis=1)

        occ = np.zeros(len(cells), bool)
        for z in np.arange(z_min, z_max + 1e-9, 0.1):
            pts = np.column_stack([cells, np.full(len(cells), z)])
            occ |= self.inside_any(pts)
        occ = occ.reshape(ny, nx)

        r = int(np.ceil(robot_radius / res))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disk = (xx ** 2 + yy ** 2) <= r ** 2
        self.grid = occ
        self.grid_inflated = binary_dilation(occ, structure=disk)
        self.origin = lo
        self.res = res
        return self.grid, self.grid_inflated

    def world_to_cell(self, xy):
        ij = np.floor((np.asarray(xy) - self.origin) / self.res).astype(int)
        return ij[:, 1], ij[:, 0]                # (row, col)

    def is_free(self, xy, inflated=True):
        g = self.grid_inflated if inflated else self.grid
        r, c = self.world_to_cell(np.atleast_2d(xy))
        ok = (r >= 0) & (r < g.shape[0]) & (c >= 0) & (c < g.shape[1])
        out = np.zeros(len(r), bool)
        out[ok] = ~g[r[ok], c[ok]]
        return out

    # ---- io --------------------------------------------------------------------
    def save(self, path):
        np.savez_compressed(
            path, names=np.array(self.names), centers=self.centers, Rs=self.Rs,
            halves=self.halves, surface=self.surface,
            grid=self.grid, grid_inflated=self.grid_inflated,
            origin=self.origin, res=self.res)

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        gt = cls(d["names"], d["centers"], d["Rs"], d["halves"])
        gt.surface, gt.grid, gt.grid_inflated = d["surface"], d["grid"], d["grid_inflated"]
        gt.origin, gt.res = d["origin"], float(d["res"])
        return gt


def from_world(world, exclude=(), models_dir=None):
    boxes = load_boxes(world, exclude, models_dir)
    names, centers, Rs, halves = zip(*boxes)
    return GroundTruth(names, centers, Rs, halves)


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world")
    ap.add_argument("--out", required=True)
    ap.add_argument("--exclude", nargs="*", default=[], help="model names to leave out")
    ap.add_argument("--models-dir", default=None, help="folder for <include> model:// lookups")
    ap.add_argument("--step", type=float, default=0.05, help="surface sample spacing [m]")
    ap.add_argument("--res", type=float, default=0.05, help="grid resolution [m]")
    ap.add_argument("--robot-radius", type=float, default=0.3)
    ap.add_argument("--z-min", type=float, default=0.05, help="grid slice lower bound")
    ap.add_argument("--z-max", type=float, default=2.0, help="grid slice upper bound")
    ap.add_argument("--preview", default=None, help="save a PNG preview here")
    a = ap.parse_args()

    gt = from_world(a.world, a.exclude, a.models_dir)
    gt.sample_surface(a.step)
    gt.build_grid(a.res, a.z_min, a.z_max, a.robot_radius)
    gt.save(a.out)

    print(f"{len(gt.names)} boxes: {sorted(set(gt.names))}")
    print(f"surface points: {len(gt.surface)}")
    print(f"grid: {gt.grid.shape} @ {gt.res} m, origin {gt.origin}, "
          f"occupied {gt.grid.mean():.1%}, inflated {gt.grid_inflated.mean():.1%}")
    print(f"saved {a.out}")

    if a.preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ext = [gt.origin[0], gt.origin[0] + gt.grid.shape[1] * gt.res,
               gt.origin[1], gt.origin[1] + gt.grid.shape[0] * gt.res]
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(gt.grid_inflated, origin="lower", extent=ext, cmap="Greys", alpha=0.35)
        ax.imshow(gt.grid, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal")
        fig.savefig(a.preview, dpi=120, bbox_inches="tight")
        print(f"preview {a.preview}")


if __name__ == "__main__":
    main()