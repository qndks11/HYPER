# src/sensing — notes for Claude

Driver packages for the physical car: `hyper_rtk`, `hyper_ebimu`, `hyper_camera`, `hyper_lidar`,
`ntrip_client`, `ublox`, `witmotion_ros2`. Each has its own `README.md`.

- **Nothing here is verifiable in Gazebo.** In simulation these nodes don't run at all — Gazebo
  publishes `/scan`, `/gps/fix`, `/imu`, `/camera/image_raw` directly. A change in this subtree
  is only confirmed with the real sensor connected on the bench; "it builds and sim still runs"
  proves nothing about it.
- **Absolute yaw comes only from the dual-GNSS moving-base RTK baseline** (`hyper_rtk`'s rover
  node publishing `/imu/heading`), and only while RTK is FIXED. There is no magnetometer heading
  and no survey constant to fall back on — the EBIMU contributes roll/pitch and gyro-z only
  (`imu0_config` index 5 is false in `hyper_localization/config/dual_ekf_navsat.yaml`).
- Serial devices are addressed through the fixed udev names (`/dev/video_logitech` and friends)
  set up in the root `README.md` — never hard-code `/dev/ttyUSB<n>` or `/dev/video<n>`.
