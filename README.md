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

Sensor protobuf timestamps are used by default. The sonar keeps an arbitrary
clock until NTP succeeds, so each timestamped message is compared with ROS
receive time. When they differ by more than `max_sensor_clock_offset` (1 s by
default), the message is re-anchored to receive time, and IMU batches keep
their sample spacing. The switch is logged in both directions, and diagnostics
report `timestamp_source`, `sensor_clock_offset_seconds` and
`sensor_clock_fallbacks`. Give the sonar a reachable NTP server so its own
acquisition timestamps are used. Set `use_sensor_timestamps:=false` to always
stamp on receipt.

The driver and player build each enabled product only while that topic has a
subscriber, including subscribers in the same component process. Subscription
changes take effect as discovery completes. Packet reception and live stream
diagnostics continue when no data topics have subscribers.

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
| `udp_receive_buffer_size` | `1048576` | Requested socket receive buffer in bytes; zero keeps the OS default |
| `poll_period` | `0.01` | Nonblocking socket poll period in wall-clock seconds |
| `max_packets_per_spin` | `32` | Maximum datagrams drained per executor callback |
| `use_sensor_timestamps` | `true` | Prefer valid device timestamps over receive time |
| `max_sensor_clock_offset` | `1.0` | Seconds a sensor timestamp may differ from receive time before the message is re-anchored to receive time; zero trusts the sensor clock unconditionally |
| `publish_point_cloud` | `true` | Enable the derived cloud |
| `publish_range_image` | `true` | Enable the scaled range image |
| `publish_bitmap_images` | `true` | Enable signal-strength and shaded bitmap topics |
| `publish_imu` | `true` | Publish `ImuBatch` samples when present |
| `diagnostics_period` | `1.0` | Diagnostic publication period in seconds |
| `packet_stale_timeout` | `2.0` | Time without valid packets before diagnostics report a stale stream |

Parameters that determine socket, publisher, or device setup are read-only;
restart the node to change them. This avoids reporting a parameter update that
was not actually applied to the hardware or transport.

The receive buffer holds packet bursts while the executor is busy. The OS can
adjust or clamp the request; `/diagnostics` reports the effective socket value
as `udp_receive_buffer_bytes`. This setting changes only the driver's socket.

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

Playback uses deadlines anchored to the first valid recorded timestamp, so
decoding and publication consume the recorded interval instead of extending it.
Repeated timestamps and older interleaved IMU samples are sent without an extra
delay. A backward clock jump exceeding one second starts a new timing segment.
`--realtime-factor` scales these intervals; `--receive-time` changes ROS message
stamps while retaining recorded pacing. Start subscribers (including rosbag)
before playback and allow enough startup delay for discovery.

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
It also exercises full-resolution dense/sparse conversions, UDP burst retention,
single-sonar RIP1/RIP2 delivery through DDS and intra-process subscriptions,
subscriber join/leave, and mixed recording playback with both timestamp modes.
Deterministic clock tests cover processing time, interleaved IMU batches, speed
factors, and recorded clock resets. Live stream tests check that an
unsynchronized sonar clock is re-anchored to receive time with IMU spacing
preserved, and that a synchronized clock keeps exact sensor timestamps.

For repeatable performance measurements, build the optional benchmark from the
workspace root after an optimized build with testing enabled:

```bash
colcon build --packages-select sonar3d --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
cmake --build build/sonar3d --target benchmark_stream
./build/sonar3d/benchmark_stream conversions
./build/sonar3d/benchmark_stream live 20 200 all 2 dds
```

The live arguments are rate (`5` or `20` Hz), frame count, subscribed products
(`none`, `range`, `cloud`, `all`), image protocol version (`1` or `2`), and ROS
transport (`dds` or `ipc`). It simulates one sonar over loopback multicast with
256×64 images and 100 Hz RIP2 IMU samples. It reports delivered counts, sequence
gaps, range/cloud latency, and CPU for the entire test process, including the
sender and subscriber. HTTP configuration is disabled. Nonzero exit status
indicates incomplete delivery; timing values are measurements, not pass limits.
Use an unused `ROS_DOMAIN_ID` to isolate these ROS topics from other tests.

## Protocol source and license

The wire definition and behavior follow Water Linked's
[Sonar 3D-15 integration API](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/)
and the tagged
[`wlsonar` 0.5.4 implementation](https://github.com/waterlinked/wlsonar/tree/v0.5.4).
Both the driver and vendored protocol definition are MIT licensed. Recording
playback history retains attribution to Marios Xanthidis (SINTEF Ocean), the
Research Council of Norway EchoNav project, and Alberto Quattrini Li's earlier
ROS integration.
