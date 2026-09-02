# Sonar 3D-15 ROS 2 Driver

Native C++20 ROS 2 driver and recording player for the Water Linked Sonar
3D-15. The package is tested on ROS 2 Jazzy and has no Python or `wlsonar`
runtime dependency.

The driver is split into small reusable libraries:

- `sonar3d_protocol` validates CRC and framing, decompresses RIP2 with Snappy,
  and decodes the vendor protobuf. RIP1 remains supported for old recordings.
- `sonar3d_ros` validates images and converts them to ROS messages and REP-103
  geometry.
- `sonar3d_io` owns the nonblocking multicast socket and bounded HTTP client.
- `sonar3d_component` is a composable `rclcpp` node. The installed
  `sonar_publisher` executable loads that same component.

The protobuf definition is generated during the CMake build from Water
Linked's published protocol, so generated sources are not checked in.

## Published topics

All topic names are relative and can be namespaced or remapped normally.

| Topic | Type | Contract |
| --- | --- | --- |
| `sonar_range_image` | `sensor_msgs/Image` | `32FC1` range in metres; zero means no return |
| `sonar_intensity_image` | `sensor_msgs/Image` | Native `mono8` signal-strength bitmap |
| `sonar_shaded_image` | `sensor_msgs/Image` | Native `mono8` vendor shaded-depth bitmap |
| `sonar_point_cloud` | `sensor_msgs/PointCloud2` | Valid returns with `x,y,z,range,azimuth,elevation` float32 fields |
| `sonar_imu` | `sensor_msgs/Imu` | Batched specific force and angular rate, one ROS message per sample |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Stream health, configuration state, CRC errors, and sequence statistics |

The point cloud is x-forward, y-left, z-up (REP-103). The device's native
x-forward, y-right, z-down coordinates are converted when points and IMU
vectors are built. Images retain the vendor's row-major pixel order.

The internal IMU origin is documented as `(-0.022, -0.046, -0.003)` metres in
the vendor frame relative to the point-cloud origin. After the REP-103 axis
conversion, publish a static transform from `sonar3d_link` to
`sonar3d_imu_link` with translation `(-0.022, 0.046, 0.003)` metres for a
complete TF model.

Sensor protobuf timestamps are used by default. Set
`use_sensor_timestamps:=false` if the sonar clock is not synchronized to the
ROS system clock.

## Dependencies and build

Install dependencies through rosdep; no pip step is needed:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sonar3d
source install/setup.bash
```

The native protocol implementation requires Protobuf, Snappy, and zlib. The
HTTP configuration client uses libcurl.

## Live operation

```bash
ros2 launch sonar3d sonar3d.launch.py \
  sonar_ip:=192.168.194.96 \
  frame_id:=sonar3d_link \
  multicast_interface:=0.0.0.0
```

HTTP configuration runs on a bounded background thread, so a missing sonar
does not block the executor or multicast receive path. Setting
`configure_sonar:=false` makes the node listen without changing device state.

The component can also be loaded into an existing container:

```bash
ros2 component load /ComponentManager sonar3d sonar3d::SonarDriver
```

The checked-in defaults are in `src/sonar3d/config/sonar3d.yaml`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `sonar_ip` | `192.168.194.96` | Literal IPv4 packet source and HTTP API address; empty accepts any source when configuration is disabled |
| `frame_id` | `sonar3d_link` | Frame for range, bitmap, and point-cloud products |
| `imu_frame_id` | `sonar3d_imu_link` | Frame at the internal IMU origin |
| `speed_of_sound` | `0.0` | m/s to configure; zero preserves the device setting |
| `configure_sonar` | `true` | Enable acoustics and multicast through the HTTP API |
| `http_timeout` | `5.0` | Ordinary HTTP timeout in seconds |
| `multicast_group` | `224.0.0.96` | RIP multicast group |
| `multicast_port` | `4747` | RIP UDP port |
| `multicast_interface` | `0.0.0.0` | Local IPv4 interface used to join multicast |
| `poll_period` | `0.01` | Nonblocking socket poll period in wall-clock seconds |
| `max_packets_per_spin` | `32` | Maximum datagrams drained per executor callback |
| `use_sensor_timestamps` | `true` | Prefer valid device timestamps over receive time |
| `publish_point_cloud` | `true` | Enable the derived cloud |
| `publish_range_image` | `true` | Enable the scaled range image |
| `publish_bitmap_images` | `true` | Enable signal-strength and shaded bitmap topics |
| `publish_imu` | `true` | Publish `ImuBatch` samples when present |
| `diagnostics_period` | `1.0` | Diagnostic publication period in seconds |
| `packet_stale_timeout` | `2.0` | Time without valid packets before diagnostics report a stale stream |

Parameters that determine socket, publisher, or device setup are read-only;
restart the node to change them. This avoids reporting a parameter update that
was not actually applied to the hardware or transport.

## Recording playback

Playback is native C++:

```bash
ros2 run sonar3d sonar_replay \
  --file survey.sonar \
  --realtime-factor 1.0 \
  --frame-id sonar3d_link
```

The old `sonar_to_bag` executable name remains as a compatibility alias.

It reads mixed RIP1/RIP2 recordings using declared packet lengths, validates
CRCs, skips damaged packets when framing remains recoverable, and publishes
the same data topics as the live driver with reliable QoS. A short startup
delay allows DDS discovery before the first sample; control it with
`--startup-delay`. Use `--receive-time` to ignore recorded sensor timestamps.

Record the replay with standard rosbag tooling:

```bash
ros2 bag record \
  /sonar_range_image \
  /sonar_intensity_image \
  /sonar_shaded_image \
  /sonar_point_cloud \
  /sonar_imu
```

## Tests

```bash
colcon test --packages-select sonar3d --return-code-on-test-failure
colcon test-result --verbose
```

The suite covers RIP1 and RIP2 round trips, an external golden packet produced
by official `wlsonar` 0.5.4, corrupt/truncated framing, ROS image/cloud
contracts, REP-103 geometry, IMU conversion, and configuration sequencing.

## Protocol source and license

The wire definition and behavior follow Water Linked's
[Sonar 3D-15 integration API](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/)
and the tagged
[`wlsonar` 0.5.4 implementation](https://github.com/waterlinked/wlsonar/tree/v0.5.4).
Both the driver and vendored protocol definition are MIT licensed. Recording
playback history retains attribution to Marios Xanthidis (SINTEF Ocean), the
Research Council of Norway EchoNav project, and Alberto Quattrini Li's earlier
ROS integration.
