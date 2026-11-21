#!/usr/bin/env python3

import sys
import numpy as np
import open3d as o3d


def visualize_ground_truth(path):
    data = np.load(path, allow_pickle=False)

    points = data["surface"]
    centers = data["centers"]
    rotations = data["Rs"]
    halves = data["halves"]
    names = data["names"]

    # Surface point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.1, 0.1, 0.1])

    geometries = [pcd]

    # Oriented bounding boxes
    for center, R, half in zip(centers, rotations, halves):
        bbox = o3d.geometry.OrientedBoundingBox(
            center=center,
            R=R,
            extent=2.0 * half
        )

        bbox.color = [1.0, 0.2, 0.1]
        geometries.append(bbox)

    # Coordinate frame
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0,
        origin=[0, 0, 0]
    )
    geometries.append(frame)

    print(f"Ground truth: {path}")
    print(f"Models: {sorted(set(names))}")
    print(f"Bounding boxes: {len(centers)}")
    print(f"Surface points: {len(points)}")

    o3d.visualization.draw_geometries(
        geometries,
        window_name="Ground Truth 3D Visualizer",
        width=1280,
        height=800,
        point_show_normal=False
    )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python view_gt.py <ground_truth.npz>")
        sys.exit(1)

    visualize_ground_truth(sys.argv[1])