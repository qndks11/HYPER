# hyper_imu

WitMotion BLE IMU (WT901BLE / similar) driver. Connects over Bluetooth LE
using `bleak`, parses the device's combined accel/gyro/angle notification
frames, and publishes `sensor_msgs/Imu` on `imu/data`.

## Dependencies

`bleak` is a pip package, not a rosdep-resolvable one:

```bash
pip install bleak
```

## Configuration

`config/imu_params.yaml`:

- `device_name` — substring to match when scanning (used if `device_address` is empty)
- `device_address` — BLE MAC address; if set, skips scanning and connects directly
- `service_uuid` / `notify_uuid` — WT901BLE GATT UUIDs (defaults match the common nRF52-based WT901BLE modules)
- `frame_id` — IMU frame for the published message header
- `scan_timeout` / `reconnect_wait_seconds` — BLE scan/reconnect tuning

## Run

```bash
ros2 launch hyper_imu imu.launch.py
```

## Notes

- Orientation is derived from the device's onboard roll/pitch/yaw fusion output
  (not computed on-host), converted to quaternion.
- Only the standard `0x61` (accel + gyro + angle) output frame is parsed.
  Magnetometer/quaternion frames from other WitMotion output modes are not
  currently handled.
- If your specific WT901BLE variant uses different service/notify UUIDs, override
  them in `config/imu_params.yaml`.
