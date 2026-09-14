# hyper_object_detection — notes for Claude

Read `README.md` in this directory first (the `/perception/sign` value table and
`sign_class_map`). This is the Python package of the pair; `hyper_lane_detection` is the C++ one.

- **Don't read `models/best.pt`** — it's a multi-MB binary weight file. To learn what classes it
  has, run the node and read its startup log lines `YOLO model classes:` and `Sign class map:`.
- **Class-name mismatches are a parameter fix, not a code fix.** If a retrained model renames a
  class, set `sign_class_map` at launch rather than editing `SIGNAL_MAP` in the source.
- Like the lane node, this one publishes its annotated frame as a topic instead of opening an
  `imshow` window (`object_detection_node.py:184`) — keep it that way.
- Python package: `colcon build --packages-select hyper_object_detection` and re-source before
  believing a change took effect; entry points don't hot-reload.
