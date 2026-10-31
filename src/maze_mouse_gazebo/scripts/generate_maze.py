#!/usr/bin/env python3
"""Generate a local, mesh-free SDF world and matching vector floor plans."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def child(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def passages(layout):
    edges = set()
    for path in [layout['main_path'], *layout['branches']]:
        for a, b in zip(path, path[1:]):
            a, b = tuple(a), tuple(b)
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                raise ValueError(f'Non-adjacent cells: {a}, {b}')
            if not all(0 <= c[0] < layout['columns'] and 0 <= c[1] < layout['rows']
                       for c in (a, b)):
                raise ValueError(f'Cell outside maze: {a}, {b}')
            edges.add(frozenset((a, b)))
    return edges


def wall_segments(layout):
    """Integer vertex pairs; a missing internal wall is a traversable passage."""
    columns, rows = layout['columns'], layout['rows']
    edges = passages(layout)
    walls = set()
    for y in range(rows + 1):
        for x in range(columns):
            if frozenset(((x, y - 1), (x, y))) not in edges:
                walls.add(((x, y), (x + 1, y)))
    for x in range(columns + 1):
        for y in range(rows):
            if frozenset(((x - 1, y), (x, y))) not in edges:
                walls.add(((x, y), (x, y + 1)))
    for gate in (layout['entry'], layout['exit']):
        x, y = gate['cell']
        endpoints = {
            'west': ((x, y), (x, y + 1)),
            'east': ((x + 1, y), (x + 1, y + 1)),
            'south': ((x, y), (x + 1, y)),
            'north': ((x, y + 1), (x + 1, y + 1)),
        }
        walls.remove(endpoints[gate['side']])
    return sorted(walls)


def solid(world, name, pose, shape, dimensions, color, collision=True):
    model = child(world, 'model', name=name)
    child(model, 'static', 'true')
    child(model, 'pose', ' '.join(f'{v:.6f}' for v in pose))
    link = child(model, 'link', name='link')
    for kind in (['collision', 'visual'] if collision else ['visual']):
        part = child(link, kind, name=kind)
        geometry = child(child(part, 'geometry'), shape)
        if shape == 'box':
            child(geometry, 'size', ' '.join(f'{v:.6f}' for v in dimensions))
        else:
            child(geometry, 'radius', f'{dimensions[0]:.6f}')
            child(geometry, 'length', f'{dimensions[1]:.6f}')
        if kind == 'visual':
            material = child(part, 'material')
            child(material, 'ambient', color)
            child(material, 'diffuse', color)
        elif name == 'ground_plane':
            ode = child(child(child(part, 'surface'), 'friction'), 'ode')
            child(ode, 'mu', '1.0')
            child(ode, 'mu2', '1.0')


def make_world(layout):
    sdf = ET.Element('sdf', version='1.6')
    sdf.append(ET.Comment(
        ' Generated from config/maze_layout.json. Units: metres, radians. '
        'Design geometry only; no controller or precomputed navigation map. '))
    world = child(sdf, 'world', name=layout['name'])
    child(world, 'gravity', '0 0 -9.81')
    physics = child(world, 'physics', name='default_physics', type='ode')
    child(physics, 'max_step_size', '0.001')
    child(physics, 'real_time_update_rate', '1000')
    scene = child(world, 'scene')
    child(scene, 'ambient', '0.45 0.45 0.45 1')
    child(scene, 'background', '0.82 0.87 0.92 1')
    child(scene, 'shadows', 'true')
    sun = child(world, 'light', name='sun', type='directional')
    child(sun, 'pose', '0 0 10 0 0 0')
    child(sun, 'diffuse', '0.8 0.8 0.8 1')
    child(sun, 'specular', '0.2 0.2 0.2 1')
    child(sun, 'direction', '-0.4 0.2 -1')
    child(sun, 'cast_shadows', 'true')
    gui = child(world, 'gui', fullscreen='0')
    camera = child(gui, 'camera', name='maze_camera')
    child(camera, 'pose', '3 -5.5 10 0 0.85 1.570796')
    child(camera, 'view_controller', 'orbit')

    scale = layout['cell_size']
    width, depth = layout['columns'] * scale, layout['rows'] * scale
    margin, thickness, height = (layout[k] for k in
                                  ('floor_margin', 'wall_thickness', 'wall_height'))
    solid(world, 'ground_plane', (width / 2, depth / 2, -0.1, 0, 0, 0),
          'box', (width + 2 * margin, depth + 2 * margin, 0.2), '0.75 0.78 0.82 1')
    walls = wall_segments(layout)
    for i, ((ax, ay), (bx, by)) in enumerate(walls):
        size = (scale if ax != bx else thickness,
                scale if ay != by else thickness, height)
        solid(world, f'wall_{i:03d}',
              ((ax + bx) * scale / 2, (ay + by) * scale / 2, height / 2, 0, 0, 0),
              'box', size, '0.32 0.38 0.48 1')
    # Caps close wall joints and give exposed ends a rounded edge.
    for i, (x, y) in enumerate(sorted({v for wall in walls for v in wall})):
        solid(world, f'pillar_{i:03d}', (x * scale, y * scale, height / 2, 0, 0, 0),
              'cylinder', (thickness / 2, height), '0.32 0.38 0.48 1')
    # Painted pads have visuals only: they do not obstruct wheels or laser rays.
    for name, point, color in (
            ('entry', layout['spawn'], '0.15 0.75 0.45 1'),
            ('exit', layout['goal'], '0.95 0.55 0.15 1')):
        solid(world, f'marker_{name}', (point['x'], point['y'], 0.001, 0, 0, 0),
              'box', (0.45, 0.45, 0.002), color, collision=False)
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True) + '\n'


def make_reference(layout):
    """Connect corridor centres with tangent quarter circles, without stop points."""
    scale = layout['cell_size']
    vertices = [((x + 0.5) * scale, (y + 0.5) * scale) for x, y in layout['main_path']]
    vertices.append((layout['goal']['x'], layout['goal']['y']))
    points = [vertices[0]]
    spacing = layout['reference_sample_spacing']
    radii = []

    def straight(target):
        start = points[-1]
        count = max(1, math.ceil(math.dist(start, target) / spacing))
        points.extend((start[0] + (target[0] - start[0]) * t / count,
                       start[1] + (target[1] - start[1]) * t / count)
                      for t in range(1, count + 1))

    for a, b, c in zip(vertices, vertices[1:], vertices[2:]):
        incoming, outgoing = math.dist(a, b), math.dist(b, c)
        u = ((b[0] - a[0]) / incoming, (b[1] - a[1]) / incoming)
        v = ((c[0] - b[0]) / outgoing, (c[1] - b[1]) / outgoing)
        cross = u[0] * v[1] - u[1] * v[0]
        if abs(cross) < 1e-8:
            straight(b)
            continue
        radius = min(layout['reference_turn_radius'], incoming / 2, outgoing / 2)
        radii.append(radius)
        tangent = (b[0] - radius * u[0], b[1] - radius * u[1])
        center = (tangent[0] + radius * v[0], tangent[1] + radius * v[1])
        straight(tangent)
        angle = math.atan2(tangent[1] - center[1], tangent[0] - center[0])
        count = max(2, math.ceil(math.pi * radius / 2 / spacing))
        for i in range(1, count + 1):
            theta = angle + math.copysign(math.pi / 2, cross) * i / count
            points.append((center[0] + radius * math.cos(theta),
                           center[1] + radius * math.sin(theta)))
    straight(vertices[-1])
    points = [p for i, p in enumerate(points) if i == 0 or math.dist(p, points[i - 1]) > 1e-8]
    return {
        'description': 'Design reference for centre-of-corridor navigation; not a controller or measured track.',
        'coordinate_frame': 'Gazebo world, metres',
        'minimum_turn_radius_m': min(radii),
        'sample_spacing_max_m': spacing,
        'length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        'cell_center_waypoints': vertices,
        'smoothed_points': points,
    }


def make_plan(layout, solution=False):
    """Native SVG diagram, with the same wall segments as the SDF."""
    scale = 90
    n, m = layout['columns'], layout['rows']
    width, height = 860, 820

    def point(p):
        return (130 + p[0] * scale, 115 + (m - p[1]) * scale)

    def line(a, b, style):
        x1, y1 = point(a)
        x2, y2 = point(b)
        return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" {style}/>'

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#f6f8fb"/>',
           '<g font-family="sans-serif" fill="#223447">',
           '<text x="52" y="45" font-size="26" font-weight="700">MAZE MOUSE / 6 × 6</text>',
           '<text x="52" y="74" font-size="14" fill="#617285">ROS 2 Humble · TurtleBot3 Burger · Gazebo Classic</text>',
           '<rect x="85" y="90" width="630" height="610" rx="12" fill="white"/>']
    for i in range(n + 1):
        svg.append(line((i, 0), (i, m), 'stroke="#e5eaf0" stroke-dasharray="3 5"'))
    for j in range(m + 1):
        svg.append(line((0, j), (n, j), 'stroke="#e5eaf0" stroke-dasharray="3 5"'))
    if solution:
        pts = [point((x / layout['cell_size'], y / layout['cell_size']))
               for x, y in make_reference(layout)['smoothed_points']]
        svg.append('<polyline points="' + ' '.join(f'{x},{y}' for x, y in pts) +
                   '" fill="none" stroke="#4bbaa8" stroke-width="3" stroke-dasharray="6 5"/>')
    for a, b in wall_segments(layout):
        svg.append(line(a, b, f'stroke="#526779" stroke-width="{layout["wall_thickness"] * scale / layout["cell_size"]}" stroke-linecap="round"'))
    for i in range(n):
        x, y = point((i + 0.5, -0.3))
        svg.append(f'<text x="{x}" y="{y}" text-anchor="middle" font-size="13">{i + 0.5:g}</text>')
    for j in range(m):
        x, y = point((-0.3, j + 0.5))
        svg.append(f'<text x="{x}" y="{y + 4}" text-anchor="middle" font-size="13">{j + 0.5:g}</text>')
    for key, label, color in [('spawn', 'START →', '#15865e'), ('goal', 'EXIT →', '#bf6b16')]:
        p = layout[key]
        x, y = point((p['x'] / layout['cell_size'], p['y'] / layout['cell_size']))
        svg.append(f'<circle cx="{x}" cy="{y}" r="15" fill="{color}" fill-opacity="0.15"/>')
        svg.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{color}"/>')
        svg.append(f'<text x="{x}" y="{y - 24}" text-anchor="middle" font-size="12" fill="{color}">{label}</text>')
    svg.extend([
        '<text x="340" y="702" font-size="13">+x / metres →</text>',
        '<text x="45" y="100" font-size="13">+y ↑</text>',
        '<text x="52" y="742" font-size="15">6.00 m grid span · 1.00 m pitch · 0.88 m clear corridor</text>',
        '<text x="52" y="768" font-size="14" fill="#617285">0.12 m walls · 1.20 m height · Straight / L / U / Z / S turns</text>',
        '<text x="52" y="796" font-size="12" fill="#617285">' +
        ('Dashed line: smoothed centre reference. Design diagram, not a simulation capture.'
         if solution else 'Design diagram, not a simulation capture.') + '</text>',
        '</g></svg>\n',
    ])
    return '\n'.join(svg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1],
                        help='Source package directory (not the installed lib directory).')
    args = parser.parse_args()
    root = args.package_dir
    layout = json.loads((root / 'config' / 'maze_layout.json').read_text())
    outputs = {
        root / 'worlds' / 'maze_6x6.world': make_world(layout),
        root / 'docs' / 'maze_6x6.svg': make_plan(layout),
        root / 'docs' / 'maze_6x6_solution.svg': make_plan(layout, solution=True),
        root / 'config' / 'centerline_reference.json': json.dumps(make_reference(layout), indent=2) + '\n',
    }
    for path, text in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
        print(f'Generated {path}')


if __name__ == '__main__':
    main()
