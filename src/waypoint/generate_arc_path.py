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
        --lead-in -5.05 \
        --path-key reverse_parking \
        --course-yaml src/waypoint/course.yaml

Reverse example (back up 4m, then take a 3m-radius arc in reverse):
    python3 src/waypoint/generate_arc_path.py \
        --radius 4.0 \
        --lead-in 4.3 \
        --post-back 1.0 \
        --reverse \
        --path-key reverse_parking

Post-arc backing example (after the quarter-circle turn, back up an extra
2m in the direction the vehicle is then facing):
    python3 src/waypoint/generate_arc_path.py \
        --radius 8.0 \
        --post-back 2.0
"""
import argparse
import math

import yaml

SPACING = 0.15  # matches waypoint_recorder.py's resample_ds default


def build_path(radius: float, lead_in: float, spacing: float = SPACING, reverse: bool = False, post_back: float = 0.0):
    points = []
    sign = -1.0 if reverse else 1.0

    # Straight lead-in: x = 0 .. sign*lead_in, y = 0, yaw = 0
    n_lead = round(lead_in / spacing)
    for i in range(n_lead + 1):
        points.append({'x': round(sign * i * spacing, 4), 'y': 0.0, 'yaw': 0.0})

    # Quarter-circle turn of the given radius, offset so it starts where the
    # lead-in ends. Forward ends at (lead_in + R, R, +90 deg); reverse is the
    # mirror image, ending at (-lead_in - R, R, -90 deg).
    arc_length = radius * (math.pi / 2)
    n_arc = round(arc_length / spacing)
    for i in range(1, n_arc + 1):
        s = i * arc_length / n_arc
        theta = s / radius
        points.append({
            'x': round(sign * (radius * math.sin(theta) + lead_in), 4),
            'y': round(radius * (1 - math.cos(theta)), 4),
            'yaw': round(sign * theta, 5),
        })

    # Extra straight backing segment after the arc: continues from wherever
    # the arc ended, moving opposite to the heading it ended with (i.e. in
    # reverse gear), for `post_back` meters.
    end = points[-1] if points else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    yaw_end = end['yaw']
    n_back = round(post_back / spacing)
    for i in range(1, n_back + 1):
        d = i * spacing
        points.append({
            'x': round(end['x'] - d * math.cos(yaw_end), 4),
            'y': round(end['y'] - d * math.sin(yaw_end), 4),
            'yaw': round(yaw_end, 5),
        })

    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--radius', type=float, required=True, help='turn radius in meters')
    parser.add_argument('--lead-in', type=float, default=4.05, help='straight lead-in length in meters before the turn starts (default: 4.05, matching current intersection_C_left)')
    parser.add_argument('--reverse', action='store_true', help='back straight instead of driving forward, then take the arc in reverse (mirror image of the forward path)')
    parser.add_argument('--post-back', type=float, default=0.0, help='additional straight backing distance in meters after the arc completes, continuing in the direction the vehicle is then facing (default: 0, no extra backing)')
    parser.add_argument('--path-key', default='intersection_C_left', help='key under paths: to replace (default: intersection_C_left)')
    parser.add_argument('--course-yaml', default='src/waypoint/course.yaml', help='path to course.yaml (default: src/waypoint/course.yaml)')
    args = parser.parse_args()

    with open(args.course_yaml) as f:
        data = yaml.safe_load(f)

    data['paths'][args.path_key] = build_path(args.radius, args.lead_in, reverse=args.reverse, post_back=args.post_back)

    with open(args.course_yaml, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print(f"Wrote {len(data['paths'][args.path_key])} points to paths.{args.path_key} "
          f"(radius={args.radius}m, lead_in={args.lead_in}m, reverse={args.reverse}, post_back={args.post_back}m) in {args.course_yaml}")


if __name__ == '__main__':
    main()
