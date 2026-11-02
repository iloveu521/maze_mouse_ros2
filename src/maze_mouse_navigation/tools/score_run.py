#!/usr/bin/env python3
"""Summarise turning oscillation and completion from recorded ROS topics."""

import csv
import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree


def steering(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    points = []
    for row in rows:
        sample = (float(row["stamp_s"]), float(row["angular"]))
        if points and points[-1][0] == sample[0]:
            points[-1] = sample
        else:
            points.append(sample)
    if len(points) < 2:
        return {}
    changes = 0
    sign = 0
    for _, angular in points:
        direction = 1 if angular > 0.15 else -1 if angular < -0.15 else 0
        if direction:
            changes += bool(sign and sign != direction)
            sign = direction
    duration = max(0.001, points[-1][0] - points[0][0])
    total_variation = sum(
        abs(points[i][1] - points[i - 1][1]) for i in range(1, len(points))
    )
    return {"reversals": changes, "angular_variation_per_s": round(total_variation / duration, 3)}


def wall_clearance(path):
    world = (Path(__file__).resolve().parents[2] /
             "maze_mouse_gazebo/worlds/maze_6x6.world")
    if not world.exists() or not path.exists():
        return None
    shapes = []
    for model in ElementTree.parse(world).iterfind(".//model"):
        name = model.get("name", "")
        if not name.startswith(("wall_", "pillar_")):
            continue
        x, y = map(float, model.findtext("pose").split()[:2])
        box = model.findtext("link/collision/geometry/box/size")
        if box:
            sx, sy = map(float, box.split()[:2])
            shapes.append((x, y, sx / 2, sy / 2, False))
        else:
            radius = float(model.findtext("link/collision/geometry/cylinder/radius"))
            shapes.append((x, y, radius, radius, True))
    closest = float("inf")
    with path.open() as stream:
        for row in csv.DictReader(stream):
            x, y = float(row["x"]), float(row["y"])
            for cx, cy, a, b, circle in shapes:
                distance = (math.hypot(x - cx, y - cy) - a if circle else
                            math.hypot(max(0.0, abs(x - cx) - a),
                                       max(0.0, abs(y - cy) - b)))
                closest = min(closest, distance)
    return round(closest, 3)


def main():
    for name in sys.argv[1:]:
        prefix = Path(name)
        result = json.loads(prefix.with_suffix(".result.json").read_text())
        changes = {}
        status_path = prefix.with_suffix(".status.jsonl")
        if status_path.exists():
            previous = None
            with status_path.open() as stream:
                for line in stream:
                    state = json.loads(line)["state"]
                    if state != previous:
                        changes[state] = changes.get(state, 0) + 1
                        previous = state
        print(json.dumps({
            "run": name,
            "state": result["state"],
            "elapsed_sim_s": result["elapsed_sim_s"],
            "stopped_sim_s": result["stopped_sim_s"],
            "distance_m": result["odom_distance_m"],
            "min_wall_clearance_m": wall_clearance(prefix.with_suffix(".odom.csv")),
            "recoveries": changes.get("RECOVER_CLEARANCE", 0),
            "blocked": changes.get("BLOCKED", 0),
            "cmd": steering(prefix.with_suffix(".commands.csv")),
            "odom": steering(prefix.with_suffix(".odom.csv")),
        }, ensure_ascii=False))


if __name__ == "__main__":
    main()
