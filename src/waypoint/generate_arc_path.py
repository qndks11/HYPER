#!/usr/bin/env python3
"""Regenerate a straight-then-quarter-circle path in course.yaml.

Splices a new path (a straight lead-in followed by a quarter-circle turn of
radius R) into the top-level `paths:` map, without disturbing anything else
in the file (GPS fixes, other events/paths, key order) — same approach as
README.md "Method 2".

Usage:
    python3 src/waypoint/generate_arc_path.py --radius 8.0

Other inputs (all optional, shown with their current intersection_C_left
values as defaults):
    python3 src/waypoint/generate_arc_path.py \
        --radius 8.0 \
        --lead-in 5.05 \
        --path-key intersection_C_left \
        --course-yaml src/waypoint/course.yaml
"""
import argparse
import math

import yaml

SPACING = 0.15  # matches waypoint_recorder.py's resample_ds default


def build_path(radius: float, lead_in: float, spacing: float = SPACING):
    points = []

    # Straight lead-in: x = 0 .. lead_in, y = 0, yaw = 0
    n_lead = round(lead_in / spacing)
    for i in range(n_lead + 1):
        points.append({'x': round(i * spacing, 4), 'y': 0.0, 'yaw': 0.0})

    # Quarter-circle left turn of the given radius, offset so it starts
    # where the lead-in ends. Ends at (lead_in + R, R, +90 deg).
    arc_length = radius * (math.pi / 2)
    n_arc = round(arc_length / spacing)
    for i in range(1, n_arc + 1):
        s = i * arc_length / n_arc
        theta = s / radius
        points.append({
            'x': round(radius * math.sin(theta) + lead_in, 4),
            'y': round(radius * (1 - math.cos(theta)), 4),
            'yaw': round(theta, 5),
        })

    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--radius', type=float, required=True, help='turn radius in meters')
    parser.add_argument('--lead-in', type=float, default=4.05, help='straight lead-in length in meters before the turn starts (default: 4.05, matching current intersection_C_left)')
    parser.add_argument('--path-key', default='intersection_C_left', help='key under paths: to replace (default: intersection_C_left)')
    parser.add_argument('--course-yaml', default='src/waypoint/course.yaml', help='path to course.yaml (default: src/waypoint/course.yaml)')
    args = parser.parse_args()

    with open(args.course_yaml) as f:
        data = yaml.safe_load(f)

    data['paths'][args.path_key] = build_path(args.radius, args.lead_in)

    with open(args.course_yaml, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print(f"Wrote {len(data['paths'][args.path_key])} points to paths.{args.path_key} "
          f"(radius={args.radius}m, lead_in={args.lead_in}m) in {args.course_yaml}")


if __name__ == '__main__':
    main()
