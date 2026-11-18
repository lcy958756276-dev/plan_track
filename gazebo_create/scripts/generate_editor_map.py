#!/usr/bin/env python3
"""Generate matching Gazebo and ROS map files from a ros-map-editor-v2 JSON map."""

import argparse
import json
import math
from pathlib import Path


WALL_THICKNESS = 0.05
WALL_HEIGHT = 0.40
OBSTACLE_HEIGHT = 0.30


def model(name, x, y, size_x, size_y, yaw=0.0, cylinder=False, height=OBSTACLE_HEIGHT):
    if cylinder:
        geometry = (
            f"<cylinder><radius>{size_x / 2:.6f}</radius>"
            f"<length>{height:.6f}</length></cylinder>"
        )
        color = "0.20 0.35 0.80 1"
    else:
        geometry = (
            f"<box><size>{size_x:.6f} {size_y:.6f} "
            f"{height:.6f}</size></box>"
        )
        color = "0.75 0.20 0.10 1"
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {height / 2:.6f} 0 0 {yaw:.9f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''


def world_text(data, source_name):
    width, height = float(data["width"]), float(data["height"])
    parts = [f'''<?xml version="1.0"?>
<sdf version="1.6">
  <!-- Generated from {source_name}; editor coordinates use the lower-left origin. -->
  <world name="map_project2">
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode"><max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>''']
    if data.get("border", False):
        parts.extend([
            model("boundary_left", -width / 2, 0, WALL_THICKNESS, height,
                  height=WALL_HEIGHT),
            model("boundary_right", width / 2, 0, WALL_THICKNESS, height,
                  height=WALL_HEIGHT),
            model("boundary_bottom", 0, -height / 2, width, WALL_THICKNESS,
                  height=WALL_HEIGHT),
            model("boundary_top", 0, height / 2, width, WALL_THICKNESS,
                  height=WALL_HEIGHT),
        ])
    for obj in data.get("objects", []):
        x = float(obj["x"]) - width / 2
        y = float(obj["y"]) - height / 2
        kind = obj["kind"]
        if kind == "circle":
            parts.append(model(f"obstacle_{obj['id']}", x, y, float(obj["w"]), 0,
                               cylinder=True))
        elif kind == "rect":
            parts.append(model(f"obstacle_{obj['id']}", x, y, float(obj["w"]),
                               float(obj["h"]), math.radians(float(obj.get("rot", 0)))))
        else:
            raise ValueError(f"Unsupported object kind: {kind}")
    parts.append("  </world>\n</sdf>\n")
    return "\n".join(parts)


def inside_obstacle(local_x, local_y, obj):
    dx, dy = local_x - float(obj["x"]), local_y - float(obj["y"])
    if obj["kind"] == "circle":
        return dx * dx + dy * dy <= (float(obj["w"]) / 2) ** 2
    angle = math.radians(float(obj.get("rot", 0)))
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    body_x = cos_a * dx + sin_a * dy
    body_y = -sin_a * dx + cos_a * dy
    return abs(body_x) <= float(obj["w"]) / 2 and abs(body_y) <= float(obj["h"]) / 2


def write_pgm(data, path):
    width, height, resolution = float(data["width"]), float(data["height"]), float(data["resolution"])
    columns, rows = round(width / resolution), round(height / resolution)
    pixels = bytearray([254]) * (columns * rows)
    for row in range(rows):
        local_y = height - (row + 0.5) * resolution
        for column in range(columns):
            local_x = (column + 0.5) * resolution
            is_border = data.get("border", False) and (
                local_x < WALL_THICKNESS or local_x > width - WALL_THICKNESS or
                local_y < WALL_THICKNESS or local_y > height - WALL_THICKNESS)
            if is_border or any(inside_obstacle(local_x, local_y, obj)
                                for obj in data.get("objects", [])):
                pixels[row * columns + column] = 0
    path.write_bytes(f"P5\n{columns} {rows}\n255\n".encode("ascii") + pixels)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json_map", type=Path)
    parser.add_argument("--name", default="map_project2")
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    data = json.loads(args.json_map.read_text(encoding="utf-8"))
    if data.get("format") != "ros-map-editor-v2":
        raise ValueError("Expected a ros-map-editor-v2 JSON file")
    world_path = args.repo / "worlds" / f"{args.name}.world"
    map_dir = args.repo / "maps"
    pgm_path = map_dir / f"{args.name}.pgm"
    yaml_path = map_dir / f"{args.name}.yaml"
    world_path.write_text(world_text(data, args.json_map.name), encoding="utf-8")
    write_pgm(data, pgm_path)
    origin_x, origin_y = -float(data["width"]) / 2, -float(data["height"]) / 2
    yaml_path.write_text(
        f"image: {pgm_path.name}\nresolution: {float(data['resolution']):.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8")
    print(f"Generated {world_path}\nGenerated {pgm_path}\nGenerated {yaml_path}")


if __name__ == "__main__":
    main()
