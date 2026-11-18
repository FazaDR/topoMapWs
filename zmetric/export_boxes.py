# export_boxes.py   (run: python export_boxes.py)
import sys
sys.path.insert(0, "/home/faza/topoMapWs/zmetric")
from ground_truth import load_boxes

WORLD_DIR = "/home/faza/topoMapWs/src/scenarios/worlds/"
for world in ("winding_room", "divider_room"):
    boxes = load_boxes(WORLD_DIR + world + ".world")       # all boxes, incl. disappearing_wall
    with open(WORLD_DIR + world + ".boxes", "w") as f:
        f.write("# name cx cy cz  R(row-major, 9)  hx hy hz\n")
        for name, c, R, h in boxes:
            vals = list(c) + list(R.reshape(-1)) + list(h)
            f.write(name + " " + " ".join(f"{v:.6f}" for v in vals) + "\n")
    print(world, len(boxes), "boxes")