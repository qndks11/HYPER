# hyper_lane_detection — notes for Claude

Read `README.md` in this directory first (input backends, ground projection, drivable area).

- **Never add a `cv::imshow` / `cv::namedWindow` debug window.** It is a deliberate design
  decision, documented at `src/lane_detection_node.cpp:550`: a GUI window pins the node to a
  local X display the real (headless) car doesn't have, and kills the whole node on unrelated
  toolkit failures. Debug output goes to `/lane/bev/image_raw` and `/lane/bev/points`, viewed in
  RViz2's Image display — publishing is skipped when nobody subscribes, so it costs nothing.
- **Detection logic lives in `process_frame()` only.** Both input backends (`intra_process` for
  the real car, `ros_raw` for Gazebo) share that function and the same `raw_image_callback()`;
  they differ only in who publishes the topic and which camera's projection parameters are used.
  A fix belongs in the shared function, not per-backend.
- Neither backend rectifies — the real camera has no calibration file and both model an ideal
  pinhole. Don't introduce rectification on one side only.
