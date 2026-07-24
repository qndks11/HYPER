# Course event paths (`course.yaml`)

`course.yaml` in this folder defines the course events (intersections, hill stops, the
accel/obstacle zone) that the behavior supervisor reacts to. It's read at startup via the
`event_path_yaml` param in `src/total/parking_cpp_t8_bundle/config/parking_params.yaml`
(currently pointed at `~/HYPER/src/waypoint/course.yaml`) — this folder is legacy/not a built ROS 2
package, but this file is still the live data source for the current C++ pipeline.

Top-level structure:

```yaml
events:
  intersection_C:
    event_type: intersection      # or hill_stop / accel_obstacle
    latitude: ...                 # GPS fix used to trigger approach_radius_m
    longitude: ...
    approach_radius_m: 5.0
    signal_required: true
    paths:
      left: intersection_C_left   # points at a key under the top-level paths: map
paths:
  intersection_C_left:
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - ...
```

`event_type`, GPS-based triggering, and each event's `paths` dict are all read generically by
`behavior_supervisor_with_parking.cpp` — there's no per-event-ID special-casing anywhere in the
C++, so adding a new intersection or a new direction (`left`/`right`/`straight`) for an existing
one only ever requires editing this one file.

## Coordinate convention

Declared at the top of the file: `x: forward, y: left, yaw: relative_radians`. Each path is stored
relative to its own start pose, which is always `(0, 0, 0)`.

**Positive yaw = a left turn**, and a left turn naturally drifts toward positive x (forward) *and*
positive y (left) — e.g. a clean quarter-circle left turn of radius `R` ends at exactly
`(R, R, +90°)`. This was confirmed by cross-checking `vehicle_controller.cpp` (which documents
`steering_angle_ > 0.0` as steering left) against `waypoint_recorder.py`'s `pose_to_local()`
rotation (the function that produces every recorded path's local x/y/yaw) — both use the same
standard CCW-positive yaw convention. Get this sign backwards and a "left" path curves right (and
vice versa) — this repo has had left/right mix-ups before, so double check against a known-good
path (e.g. `intersection_A_straight`) if unsure.

## Method 1 — record it live in Gazebo (preferred)

`waypoint_recorder.py` (also duplicated under `src/total/parking_cpp_t8_bundle/`, but **run the
copy in this folder** — the other copy's default `output` param resolves relative to its own
directory and will silently write a second, unused `course.yaml` instead of the one
`parking_params.yaml` actually points at) subscribes to `/gps/fix` and `/odometry/filtered_map`,
and listens for text commands on `/waypoint/cmd`:

```bash
source install/setup.bash
python3 src/waypoint/waypoint_recorder.py
```

Typical flow for a new intersection direction:

```bash
# drive to the desired approach/trigger point, then:
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'mark_event:intersection_C'}"

# drive to the recording start point (becomes local origin), then:
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'record_start:intersection_C:left'}"

# drive through the maneuver, then:
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'record_end'}"

ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_radius:intersection_C:5.0'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'save'}"
```

See the recorder's own docstring (top of the file, in Korean) for the full command list
(`mark_stopline`, `mark_accel_start`/`end`, `delete_event`, `list`, `status`, ...). A clean `Ctrl-C`
also auto-saves. `waypoint_view.py` in this folder visualizes the saved events/paths in RViz.

## Method 2 — generate a path programmatically

When recording isn't practical (or didn't produce usable data), a path can be written directly.
This is what was used to create `intersection_C_left`: a quarter-circle arc of a chosen radius `R`,
parametrized by arc length and resampled at the same ~0.15 m spacing the recorder uses
(`resample_ds`), with `x`/`y` rounded to 4 decimals and `yaw` to 5 — matching the recorder's own
`process_path()` rounding so generated and recorded paths look the same on disk:

```python
import math

R = 10.0        # turn radius, meters
spacing = 0.15  # match waypoint_recorder.py's resample_ds default

arc_length = R * (math.pi / 2)   # quarter circle
n = round(arc_length / spacing)

points = []
for i in range(n + 1):
    s = i * arc_length / n
    theta = s / R
    points.append({
        'x': round(R * math.sin(theta), 4),
        'y': round(R * (1 - math.cos(theta)), 4),
        'yaw': round(theta, 5),
    })
# points[0] == (0, 0, 0); points[-1] == (R, R, +90 deg) for a left turn
```

To splice a generated path into `course.yaml` without disturbing anything else (GPS fixes, other
events/paths, formatting), load the whole file with `yaml.safe_load`, replace only
`data['paths']['<key>']`, then `yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)` —
this preserves key order and only touches the one path you're replacing.

## Gotchas

- An event's `paths` entry (e.g. `paths: {left: intersection_C_left}`) is just a dict key — it
  doesn't validate that `intersection_C_left` actually exists under the top-level `paths:` map. A
  dangling reference like this fails silently at runtime (the supervisor just won't find a path to
  drive), not at load time.
- `intersection_A_straight`/`intersection_B_straight` (and any freshly recorded path) may include a
  loop-back/near-180° section partway through if the driver looped the vehicle around before
  calling `record_end` — that's expected from how this particular track was recorded, not a bug in
  the data.
