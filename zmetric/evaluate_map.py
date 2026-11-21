#!/usr/bin/env python3

import csv
import json
import numpy as np

from pathlib import Path
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


# ============================================================
# CONFIGURATION
# ============================================================

MAP_DIR = Path(
    "/home/faza/topoMapWs/zmapping_out/mlatc_20261119_041424"
)

GT_PATH = Path(
    "/home/faza/topoMapWs/zmetric/gt_winding.npz"
)

NODE_THRESHOLD = 0.25       # Maximum node-to-surface distance (m)
NEIGHBOR_COUNT = 4          # KNN candidate connections
MAX_DETOUR_RATIO = 1.5      # Maximum alternative path / direct distance
SEGMENT_SHRINK = 0.02       # Obstacle intersection tolerance

OUTPUT_JSON = MAP_DIR / "evaluation.json"
OUTPUT_NODE_CSV = MAP_DIR / "node_details"


# ============================================================
# CSV LOADING
# ============================================================

def load_nodes(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    ids = np.array(
        [int(r["id"]) for r in rows],
        dtype=np.int64,
    )

    xyz = np.array(
        [[float(r["x"]), float(r["y"]), float(r["z"])]
         for r in rows],
        dtype=np.float64,
    ).reshape(-1, 3)

    if len(np.unique(ids)) != len(ids):
        raise ValueError(f"Duplicate node IDs: {path}")

    return ids, xyz


def load_edges(path):
    if not path.exists():
        raise FileNotFoundError(path)

    edges = set()

    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            a = int(row["a"])
            b = int(row["b"])

            if a != b:
                edges.add((min(a, b), max(a, b)))

    return edges


# ============================================================
# STATISTICS
# ============================================================

def stats(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p90": None,
            "p95": None,
        }

    return {
        "count": int(len(values)),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
    }


# ============================================================
# NODE EVALUATION
# ============================================================

def evaluate_nodes(gt, ids, xyz):

    node_dist = gt.dist_to_surface(xyz)
    inside = gt.inside_any(xyz)

    within = node_dist <= NODE_THRESHOLD

    result = {
        "surface_distance_m": stats(node_dist),

        "clearance_threshold": {
            "threshold_m": NODE_THRESHOLD,
            "within_count": int(np.count_nonzero(within)),
            "outside_count": int(np.count_nonzero(~within)),
            "within_percent": float(np.mean(within) * 100),
            "outside_percent": float(np.mean(~within) * 100),
        },

        "inside_obstacles": {
            "count": int(np.count_nonzero(inside)),
            "percent": float(np.mean(inside) * 100),
        },
    }

    return result, node_dist, inside


def save_node_details(path, ids, xyz, node_dist, inside, degrees):

    with path.open("w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "id",
            "x",
            "y",
            "z",
            "surface_distance",
            "within_threshold",
            "inside_obstacle",
            "degree",
        ])

        for i in range(len(ids)):
            writer.writerow([
                int(ids[i]),
                *xyz[i].tolist(),
                float(node_dist[i]),
                int(node_dist[i] <= NODE_THRESHOLD),
                int(inside[i]),
                int(degrees[i]),
            ])


# ============================================================
# GRAPH CONSTRUCTION
# ============================================================

def build_graph(gt, ids, xyz, node_dist, edges):

    id_to_index = {
        int(node_id): i
        for i, node_id in enumerate(ids)
    }

    adjacency = [[] for _ in range(len(ids))]
    valid_edges = set()

    invalid_references = 0
    invalid_geometry = 0

    for a, b in edges:

        ia = id_to_index.get(a)
        ib = id_to_index.get(b)

        if ia is None or ib is None:
            invalid_references += 1
            continue

        # Only consider edges between acceptable nodes.
        if (
            node_dist[ia] > NODE_THRESHOLD
            or node_dist[ib] > NODE_THRESHOLD
        ):
            invalid_geometry += 1
            continue

        pa = xyz[ia]
        pb = xyz[ib]

        # Exclude connections crossing obstacles.
        if gt.segment_hits_box(
            pa, pb, shrink=SEGMENT_SHRINK
        ):
            invalid_geometry += 1
            continue

        distance = float(np.linalg.norm(pa - pb))

        adjacency[ia].append((ib, distance))
        adjacency[ib].append((ia, distance))

        valid_edges.add((min(a, b), max(a, b)))

    return (
        adjacency,
        id_to_index,
        valid_edges,
        invalid_references,
        invalid_geometry,
    )


# ============================================================
# ALTERNATIVE PATH SEARCH
# ============================================================

def shortest_path_with_limit(
    adjacency, start, goal, max_distance
):
    """
    Dijkstra search.

    Returns shortest path distance if it is within max_distance.
    Returns None if no acceptable alternative path exists.
    """

    import heapq

    queue = [(0.0, start)]
    distances = {start: 0.0}

    while queue:

        distance, node = heapq.heappop(queue)

        if distance > max_distance:
            break

        if node == goal:
            return distance

        if distance > distances.get(node, float("inf")):
            continue

        for neighbor, weight in adjacency[node]:

            new_distance = distance + weight

            if new_distance > max_distance:
                continue

            if new_distance < distances.get(
                neighbor, float("inf")
            ):
                distances[neighbor] = new_distance

                heapq.heappush(
                    queue, (new_distance, neighbor)
                )

    return None


# ============================================================
# EDGE EVALUATION
# ============================================================

def evaluate_edges(
    gt, ids, xyz, node_dist, edges
):

    n = len(ids)

    (
        adjacency,
        id_to_index,
        valid_edges,
        invalid_references,
        invalid_geometry,
    ) = build_graph(
        gt, ids, xyz, node_dist, edges
    )

    # --------------------------------------------------------
    # Generate expected local connections using KNN
    # --------------------------------------------------------

    candidates = set()
    tree = cKDTree(xyz)

    if n > 1:

        k = min(NEIGHBOR_COUNT + 1, n)

        _, neighbors = tree.query(xyz, k=k)

        if k == 1:
            neighbors = neighbors[:, None]

        for i, row in enumerate(neighbors):

            for j in row[1:]:

                j = int(j)

                if i == j:
                    continue

                if (
                    node_dist[i] > NODE_THRESHOLD
                    or node_dist[j] > NODE_THRESHOLD
                ):
                    continue

                if gt.segment_hits_box(
                    xyz[i],
                    xyz[j],
                    shrink=SEGMENT_SHRINK,
                ):
                    continue

                a = int(ids[i])
                b = int(ids[j])

                candidates.add((min(a, b), max(a, b)))

    # --------------------------------------------------------
    # Classify candidate connections
    # --------------------------------------------------------

    direct = 0
    indirect = 0
    missing = 0

    detour_ratios = []
    indirect_examples = []

    for a, b in candidates:

        # Direct connection exists and is geometrically valid.
        if (a, b) in valid_edges:
            direct += 1
            continue

        ia = id_to_index[a]
        ib = id_to_index[b]

        euclidean_distance = float(
            np.linalg.norm(xyz[ia] - xyz[ib])
        )

        if euclidean_distance <= 1e-12:
            indirect += 1
            continue

        max_path_distance = (
            euclidean_distance * MAX_DETOUR_RATIO
        )

        path_distance = shortest_path_with_limit(
            adjacency,
            ia,
            ib,
            max_path_distance,
        )

        if path_distance is not None:

            indirect += 1

            ratio = path_distance / euclidean_distance
            detour_ratios.append(ratio)

            indirect_examples.append({
                "node_a": int(a),
                "node_b": int(b),
                "direct_distance_m": euclidean_distance,
                "alternative_path_m": path_distance,
                "detour_ratio": ratio,
            })

        else:
            missing += 1

    total_candidates = len(candidates)
    represented = direct + indirect

    result = {
        "actual_edges": len(edges),
        "valid_edges": len(valid_edges),
        "invalid_edge_references": invalid_references,
        "invalid_edge_geometry": invalid_geometry,

        "expected_local_connections": {
            "candidate_count": total_candidates,

            "directly_represented": direct,
            "indirectly_represented": indirect,
            "genuinely_missing": missing,

            "direct_recall_percent": (
                direct / total_candidates * 100
                if total_candidates else None
            ),

            "represented_connectivity_percent": (
                represented / total_candidates * 100
                if total_candidates else None
            ),

            "genuinely_missing_percent": (
                missing / total_candidates * 100
                if total_candidates else None
            ),
        },

        "alternative_paths": {
            "maximum_detour_ratio": MAX_DETOUR_RATIO,
            "detour_statistics": stats(detour_ratios),
            "indirect_examples": indirect_examples,
        },
    }

    # --------------------------------------------------------
    # Overall graph connectivity
    # --------------------------------------------------------

    degrees = np.array(
        [len(neighbors) for neighbors in adjacency],
        dtype=np.int64,
    )

    if n > 0:

        rows = []
        cols = []

        for i, neighbors in enumerate(adjacency):
            for j, _ in neighbors:
                rows.append(i)
                cols.append(j)

        matrix = csr_matrix(
            (
                np.ones(len(rows), dtype=np.int8),
                (rows, cols),
            ),
            shape=(n, n),
        )

        component_count, labels = connected_components(
            matrix,
            directed=False,
            return_labels=True,
        )

        component_sizes = np.bincount(labels)

        largest_component = int(component_sizes.max())

    else:
        component_count = 0
        largest_component = 0

    result["graph_connectivity"] = {
        "connected_components": int(component_count),
        "isolated_nodes": int(np.count_nonzero(degrees == 0)),
        "largest_component_nodes": largest_component,

        "largest_component_percent": (
            largest_component / n * 100
            if n else 0.0
        ),

        "average_degree": (
            float(degrees.mean()) if n else 0.0
        ),
    }

    return result, degrees


# ============================================================
# LAYER EVALUATION
# ============================================================

def evaluate_layer(gt, node_file, edge_file):

    ids, xyz = load_nodes(node_file)
    edges = load_edges(edge_file)

    result = {
        "nodes": len(ids),
        "edges": len(edges),
    }

    if len(ids) == 0:
        result["error"] = "No nodes"
        return result

    node_result, node_dist, inside = evaluate_nodes(
        gt, ids, xyz
    )

    edge_result, degrees = evaluate_edges(
        gt, ids, xyz, node_dist, edges
    )

    result["node_quality"] = node_result
    result["edge_quality"] = edge_result

    layer = node_file.stem.replace(
        "map_layer", ""
    ).replace("_nodes", "")

    node_csv = OUTPUT_NODE_CSV.with_name(
        f"{OUTPUT_NODE_CSV.name}_layer{layer}.csv"
    )

    save_node_details(
        node_csv,
        ids,
        xyz,
        node_dist,
        inside,
        degrees,
    )

    print(f"Saved node details: {node_csv}")

    return result


# ============================================================
# MAIN
# ============================================================

def main():

    import sys

    sys.path.insert(
        0,
        "/home/faza/topoMapWs/zmetric",
    )

    import ground_truth

    print("Loading ground truth...")
    gt = ground_truth.GroundTruth.load(GT_PATH)

    node_files = sorted(
        MAP_DIR.glob("map_layer*_nodes.csv")
    )

    if not node_files:
        raise FileNotFoundError(
            f"No node CSV files found in {MAP_DIR}"
        )

    report = {
        "map_directory": str(MAP_DIR),

        "settings": {
            "node_threshold_m": NODE_THRESHOLD,
            "neighbor_count": NEIGHBOR_COUNT,
            "max_detour_ratio": MAX_DETOUR_RATIO,
            "segment_shrink": SEGMENT_SHRINK,
        },

        "layers": {},
    }

    for node_file in node_files:

        layer = (
            node_file.stem
            .replace("map_layer", "")
            .replace("_nodes", "")
        )

        edge_file = MAP_DIR / (
            f"map_layer{layer}_edges.csv"
        )

        print(f"\nEvaluating layer {layer}...")

        report["layers"][layer] = evaluate_layer(
            gt,
            node_file,
            edge_file,
        )

    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2) + "\n"
    )

    print("\n======================================")
    print("EVALUATION COMPLETE")
    print("======================================")
    print(f"JSON report: {OUTPUT_JSON}")


if __name__ == "__main__":
    main()