#!/usr/bin/env python3
"""Plot recorded odometry on the supplied maze world for review."""

import csv
import sys
from pathlib import Path
from xml.etree import ElementTree

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def world_shapes():
    world = (Path(__file__).resolve().parents[2] /
             "maze_mouse_gazebo/worlds/maze_6x6.world")
    shapes = []
    for model in ElementTree.parse(world).iterfind(".//model"):
        name = model.get("name", "")
        if not name.startswith(("wall_", "pillar_")):
            continue
        x, y = map(float, model.findtext("pose").split()[:2])
        box = model.findtext("link/collision/geometry/box/size")
        if box:
            sx, sy = map(float, box.split()[:2])
            shapes.append((name, x, y, sx, sy))
        else:
            radius = float(model.findtext("link/collision/geometry/cylinder/radius"))
            shapes.append((name, x, y, 2 * radius, 2 * radius))
    return shapes


def trajectory(prefix):
    with Path(str(prefix) + ".odom.csv").open() as stream:
        return [(float(row["x"]), float(row["y"]))
                for row in csv.DictReader(stream)]


def main():
    if len(sys.argv) < 3:
        raise SystemExit("usage: plot_runs.py OUTPUT.svg LABEL=PREFIX [LABEL=PREFIX ...]")
    output = Path(sys.argv[1])
    fig, ax = plt.subplots(figsize=(8, 8), constrained_layout=True)
    for _, x, y, sx, sy in world_shapes():
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               facecolor="#34495e", edgecolor="#253746"))
    colors = ["#168a68", "#ed7b24", "#3867d6", "#bd3f83"]
    for i, spec in enumerate(sys.argv[2:]):
        label, prefix = spec.split("=", 1)
        points = trajectory(prefix)
        ax.plot([p[0] for p in points], [p[1] for p in points],
                color=colors[i % len(colors)], linewidth=1.25,
                alpha=.9, label=label)
        ax.scatter(*points[0], color=colors[i % len(colors)], s=24, zorder=5)
    ax.scatter([6.45], [5.5], color="#d33636", marker="*", s=110,
               label="goal", zorder=6)
    ax.set(xlim=(-.1, 6.9), ylim=(-.1, 6.9), aspect="equal",
           xlabel="x (m)", ylabel="y (m)",
           title="Gazebo odometry: path tracking tuning")
    ax.grid(True, alpha=.2)
    ax.legend(loc="upper left", fontsize=8)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, format="svg")
    plt.close(fig)


if __name__ == "__main__":
    main()
