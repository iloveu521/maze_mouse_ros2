#!/usr/bin/env python3
"""Check maze topology and actual SDF collision clearance without running Gazebo."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(root):
    layout = json.loads((root / 'config' / 'maze_layout.json').read_text())
    world = ET.parse(root / 'worlds' / 'maze_6x6.world').getroot().find('world')
    require(world is not None and world.get('name'), 'World name must not be empty')
    require(all(light.get('type') in ('directional', 'point', 'spot')
                for light in world.findall('light')), 'Invalid light type')
    n, m, scale = (layout[k] for k in ('columns', 'rows', 'cell_size'))
    cells = {(x, y) for x in range(n) for y in range(m)}
    graph = {cell: set() for cell in cells}
    for path in [layout['main_path'], *layout['branches']]:
        for a, b in zip(path, path[1:]):
            a, b = tuple(a), tuple(b)
            require(a in cells and b in cells, 'Out-of-bounds cell')
            require(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1, 'Non-adjacent passage')
            graph[a].add(b)
            graph[b].add(a)
    start, end = tuple(layout['entry']['cell']), tuple(layout['exit']['cell'])
    reached, queue = {start}, deque([start])
    while queue:
        for neighbor in graph[queue.popleft()] - reached:
            reached.add(neighbor)
            queue.append(neighbor)
    edges = sum(map(len, graph.values())) // 2
    require(reached == cells, 'Maze has unreachable cells')

    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    gate_directions = {'east': 0, 'north': 1, 'west': 2, 'south': 3}
    # Read actual generated collision geometry, independently of the cell graph.
    obstacles = []
    names = set()
    for model in world.findall('model'):
        name = model.get('name')
        require(name not in names, f'Duplicate model name {name}')
        names.add(name)
        require(model.findtext('static') == 'true', f'Non-static maze model {name}')
        pose = [float(v) for v in model.findtext('pose').split()]
        collisions = model.findall('link/collision')
        if name.startswith('marker_'):
            require(not collisions, 'Painted markers must not have collision bodies')
            continue
        require(collisions, f'Missing collision for {name}')
        require(all(abs(v) < 1e-8 for v in pose[3:]), 'Validator expects axis-aligned models')
        for collision in collisions:
            geometry = collision.find('geometry')
            visual = model.find('link/visual/geometry')
            require(ET.canonicalize(ET.tostring(geometry, encoding='unicode'), strip_text=True) ==
                    ET.canonicalize(ET.tostring(visual, encoding='unicode'), strip_text=True),
                    f'Visual/collision mismatch: {name}')
            if name == 'ground_plane':
                continue
            box = geometry.find('box/size')
            if box is not None:
                sx, sy, sz = map(float, box.text.split())
                require(sz == layout['wall_height'], 'Unexpected wall height')
                obstacles.append(('box', pose[0], pose[1], sx / 2, sy / 2))
            else:
                radius = float(geometry.findtext('cylinder/radius'))
                obstacles.append(('circle', pose[0], pose[1], radius, radius))

    def clearance(x, y):
        distances = []
        for kind, cx, cy, a, b in obstacles:
            if kind == 'circle':
                distance = math.hypot(x - cx, y - cy) - a
            else:
                distance = math.hypot(max(abs(x - cx) - a, 0), max(abs(y - cy) - b, 0))
            distances.append(distance)
        return min(distances)

    radius = layout['robot_clearance_radius']

    def check_segment(a, b, free):
        steps = max(1, math.ceil(math.dist(a, b) / 0.02))
        distances = [clearance(a[0] + (b[0] - a[0]) * t / steps,
                               a[1] + (b[1] - a[1]) * t / steps)
                     for t in range(steps + 1)]
        if free:
            require(min(distances) > radius, f'Robot clearance blocked between {a} and {b}')
        else:
            require(min(distances) < 0.001, f'Unexpected gap through a closed wall: {a}, {b}')

    def center(cell):
        return ((cell[0] + 0.5) * scale, (cell[1] + 0.5) * scale)

    # Every internal adjacency must agree with the exported SDF: open OR closed.
    for cell in sorted(cells):
        require(clearance(*center(cell)) > radius, f'Blocked cell center {cell}')
        for dx, dy in [(1, 0), (0, 1)]:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor in cells:
                check_segment(center(cell), center(neighbor), neighbor in graph[cell])
    for cell in cells:
        for direction, (dx, dy) in enumerate(directions):
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor not in cells:
                is_gate = any(tuple(g['cell']) == cell and gate_directions[g['side']] == direction
                              for g in (layout['entry'], layout['exit']))
                check_segment(center(cell), center(neighbor), is_gate)
    spawn = layout['spawn']
    goal = layout['goal']
    check_segment((spawn['x'], spawn['y']), center(start), True)
    check_segment(center(end), (goal['x'], goal['y']), True)
    reference = json.loads((root / 'config' / 'centerline_reference.json').read_text())
    reference_points = reference['smoothed_points']
    require(math.dist(reference_points[0], (spawn['x'], spawn['y'])) < 1e-6,
            'Reference path does not start at spawn')
    require(math.dist(reference_points[-1], (goal['x'], goal['y'])) < 1e-6,
            'Reference path does not reach goal')
    for a, b in zip(reference_points, reference_points[1:]):
        check_segment(a, b, True)
    minimum_clearance = min(clearance(*p) for p in reference_points)
    dead_ends = sum(len(v) == 1 for k, v in graph.items() if k not in (start, end))
    print(f'PASS: {len(cells)} connected cells, {edges} passages, {dead_ends} dead ends')
    print(f'PASS: SDF collisions match all passages and walls; robot radius {radius:.2f} m')
    print(f'PASS: rounded centre reference, radius {reference["minimum_turn_radius_m"]:.2f} m, '
          f'length {reference["length_m"]:.2f} m, minimum centre-to-wall distance {minimum_clearance:.3f} m')
    print('NOTE: reference clearance validation is not a physical controller simulation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    validate(args.package_dir)


if __name__ == '__main__':
    main()
