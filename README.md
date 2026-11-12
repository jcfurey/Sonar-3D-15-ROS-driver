# Sonar 3D-15 ROS 2 Driver

Native C++20 ROS 2 driver and recording player for the Water Linked Sonar
3D-15. The package is tested on ROS 2 Jazzy and has no Python or `wlsonar`
runtime dependency in its driver/player. The optional quantitative image viewer
uses NumPy and Matplotlib.

The driver is split into small reusable libraries:

- `sonar3d_protocol` validates CRC and framing, decompresses RIP2 with Snappy,
  and decodes the vendor protobuf. RIP1 remains supported for old recordings.
- `sonar3d_ros` validates images and converts them to ROS messages and REP-103
  geometry.
- `sonar3d_io` owns the multicast or unicast UDP socket and the cancellable sonar HTTP API client.
- `sonar3d_component` is a composable managed (lifecycle) node. The installed
  `sonar_publisher` executable runs that same component.

The protobuf definition is generated during the CMake build from Water
Linked's published protocol, so generated sources are not checked in.

## Published topics

Data topics are relative to the driver's namespace (`sonar3d` in the launch
files), so they appear as `/sonar3d/points` and so on, and can be namespaced
or remapped normally.

| Topic | Type | Contract |
| --- | --- | --- |
| `range_image` | `sensor_msgs/Image` | Upright `32FC1` range in metres; zero means no return |
| `intensity_image` | `sensor_msgs/Image` | Upright `mono8` signal-strength bitmap |
| `shaded_image` | `sensor_msgs/Image` | Upright `mono8` vendor shaded-depth bitmap |
| `points` | `sensor_msgs/PointCloud2` | Valid returns with `x,y,z,intensity,range,azimuth,elevation` float32 fields |
| `raw_packet` | `sonar3d/RawPacket` | Exact accepted-source RIP datagrams, including unknown messages, duplicates and damaged packets; host receipt stamp and sender address |
| `imu/data_raw` | `sensor_msgs/Imu` | REP-145 specific force and angular rate without orientation, one message per sample |
| `imu/data` | `sensor_msgs/Imu` | Opt-in experimental private vendor orientation only; local z-up reference, unverified heading, no angular velocity or acceleration |
| `image_metadata` | `sonar3d/ImageMetadata` | Exact image header, source shot ID/clock, angular geometry, acquisition settings and driver source fingerprint; one message per range/bitmap image |
| `quantitative_image` | `sensor_msgs/Image` | Optional headless renderer's display-only `rgb8` figure: labelled range and signal panels, fixed scales and no-return masks; measurements remain on the raw topics |
| `/tf_static` | `tf2_msgs/TFMessage` | Documented `frame_id` to `imu_frame_id` offset (`publish_tf`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_updater` statuses: stream health, lifecycle and configuration state, clock source, counters, range image rate, and the sonar's own status (temperature, self-check, NTP sync, firmware) |

Following REP-2003, data topics are published with `SystemDefaultsQoS`
(reliable), so reliable subscribers such as RViz's default and best-effort
`SensorDataQoS` subscribers both connect. Processing nodes should subscribe
with `SensorDataQoS`. Images are raw `sensor_msgs/Image` topics; for
compressed transport over a tether, run `image_transport`'s republisher, e.g.
`ros2 run image_transport republish raw compressed --ros-args -r
in:=/sonar3d/intensity_image -r out/compressed:=/sonar3d/intensity_image/compressed`
(`compressedDepth` handles the `32FC1` range image).

Data publishers accept standard ROS 2 startup `qos_overrides` parameters for
history, depth, and reliability. Use the fully resolved topic name, including
any remapping. For example, in the node's `ros__parameters` section:

```yaml
qos_overrides:
  /sonar3d/range_image:
    publisher:
      reliability: best_effort
      history: keep_last
      depth: 5
```

Image dimensions and IMU measurements are validated even without subscribers,
before updating sequence tracking or the shared sensor clock. Malformed input
cannot suppress a subsequent valid shot or poison the clock fit. Non-finite
IMU measurements and overflowing noise variances are rejected. A backward ROS
clock jump clears the previous clock fit, including pending step confirmation.
Warnings are throttled per component on a steady clock, so concurrent sonar
instances keep independent warning state even while ROS time is paused.

Multicast sockets receive only their configured group, including when another
listener joins a different group on the same port. UDP wait deadlines survive
interrupted system calls. Range/intensity pairing retains a newer cached signal
when an older signal arrives late, including across sequence wraparound.
Playback clears cached pairs when newer acquisition times identify a counter
restart, or image time steps backward by more than one second.
Recording playback validates supported products before pacing or pairing them,
even without subscribers; malformed future timestamps cannot stall replay.
Range overflow is rejected before images, metadata, or deferred clouds are emitted.

Topic names before version 0.5 carried a `sonar_` prefix. To keep old
consumers working, remap the new names back, e.g. `-r points:=sonar_point_cloud
-r range_image:=sonar_range_image -r imu/data_raw:=sonar_imu`. Version 0.6
renamed two parameters, changed the meaning of `speed_of_sound: 0` to match
the sonar's own API, and added `intensity` to the cloud:

| Before | Now |
| --- | --- |
| topic `sonar_point_cloud` | `points` (with `intensity`; `point_cloud_intensity: false` restores the 6-field layout) |
| topic `sonar_range_image` | `range_image` |
| topic `sonar_intensity_image` | `intensity_image` |
| topic `sonar_shaded_image` | `shaded_image` |
| topic `sonar_imu` | `imu/data_raw` |
| `multicast_port` | `udp_port` |
| `multicast_interface` | `interface_address` |
| `speed_of_sound: 0.0` (keep the device setting) | `speed_of_sound: -1.0`; `0.0` now selects the sonar's automatic speed of sound |

The point cloud is x-forward, y-left, z-up (REP-103). The device's native
x-forward, y-right, z-down coordinates are converted when points and IMU
vectors are built.

Each point's `intensity` is the linear signal strength of its return, from
the signal-strength image of the same shot: Water Linked encodes it as
`pixel = 100 log10(strength / 30)`, so `intensity = round(30 * 10^(pixel / 100))`,
about 31 to 10644, as in wlsonar's `bitmap_image_to_strength_linear`. The two
images share one pixel grid; in Water Linked's sample recording their return
masks agree on every pixel. The sonar sends a shot's range and signal images
back to back with the same sequence ID, so the cloud waits for the signal
image; if it does not arrive within 25 ms (or the next shot starts first),
the cloud is published with zero intensity and diagnostics count it in
`unpaired_range_images`. The range image itself is never delayed. Set
`point_cloud_intensity: false` to publish the cloud as soon as the range
image arrives, without the field.

The three images share one upright pixel grid: row 0 is the highest
elevation and column 0 the leftmost azimuth, as an image viewer expects.
Water Linked's protocol stores rows from the lowest elevation upward (pixel
row `py` has pitch `py / (height - 1) * fovV - fovV / 2`, positive up), so the
driver reverses the row order. Published row `r` corresponds to vendor row
`height - 1 - r`; column `c` has yaw `c / (width - 1) * fovH - fovH / 2`,
positive to the right. Each cloud point carries its own `azimuth` and
`elevation` in REP-103 sense (positive left and up).

The internal IMU origin is documented as `(-0.022, -0.046, -0.003)` metres in
the vendor frame relative to the point-cloud origin. The driver broadcasts
the REP-103 equivalent, translation `(-0.022, 0.046, 0.003)` metres from
`frame_id` to `imu_frame_id`, on `/tf_static`. Set `publish_tf:=false` if a
URDF already provides that joint.

IMU data follows REP-145: `linear_acceleration` is specific force in m/s²
(+g on z when level), `angular_velocity` is in rad/s, and
`orientation_covariance[0]` is -1 because the public protocol reports no
orientation. Covariances are zero ("unknown") unless
`linear_acceleration_stddev` and `angular_velocity_stddev` are set. The topic
uses REP-145's `imu` namespace; ROS 2 remaps whole names, so move it with
`-r imu/data_raw:=<new name>`.

IMU samples arrive in batches (5 at 20 Hz, 20 at 5 Hz) and are published back
to back with a 100-message history. Subscribe with a queue depth of at least
the batch size, for example `rclcpp::SensorDataQoS(rclcpp::KeepLast(50))`;
the default depth of 5 drops samples from 5 Hz batches.

Sensor protobuf timestamps are used by default. The sonar keeps an arbitrary
clock until NTP succeeds, so each timestamped message is compared with ROS
receive time. When the minimum observed offset exceeds `max_sensor_clock_offset` (1 s by
default), sensor timestamps are shifted onto ROS time by one offset shared by
every product: the smallest receive-minus-sensor difference seen over the last
10–20 s. Transport delay only ever makes a packet late, so this minimum removes
network and processing jitter, follows slow clock drift, and preserves the
sonar's own spacing between images and IMU samples. Stamps therefore reflect
acquisition time plus the minimum delivery delay rather than each packet's
receipt. A backward clock step must persist across at least three observations
spanning one second before the estimate restarts; isolated late packets and
queued bursts cannot reset it. The switch
is logged in both directions, and diagnostics report `timestamp_source`,
`sensor_clock_offset_seconds` and `sensor_clock_fallbacks`. Give the sonar a
reachable NTP server so its own acquisition timestamps are used. Set
`use_sensor_timestamps:=false` to always stamp on receipt.

Duplicates and older sequence IDs are rejected before clock estimation and
processed publication. A counter restart with a newer timestamp is accepted;
if both the counter and sensor clock reset, recovery requires a stale stream
and three advancing packets spanning at least 50 ms, excluding queued bursts.
`raw_packet` retains these packets unchanged.

Record `raw_packet` for complete source preservation: IMU batch IDs/counts,
native pixel ordering, unknown protobuf fields/types, framing and CRC are all
retained. Processed images, clouds and individual IMU messages are derived
views and cannot retain every source attribute. The raw header is host receipt
time in live operation; it does not claim synchronized acquisition time.
Its `packet_index` preserves input order independently of acquisition-time
sorting by a bag reader. Sort by this index before concatenating raw bytes to
reconstruct a native `.sonar` stream; receipt-index gaps can reflect source
filtering or an unsubscribed interval.

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
  interface_address:=0.0.0.0
```

The launch file loads `params_file` (default `config/sonar3d.yaml`, written for
`/**/sonar3d_driver` so it applies in any namespace). Every driver parameter
is also a launch argument; an argument overrides the file only when given, and
is converted to the parameter's declared type, so `speed_of_sound:=1491` is
accepted as a double. Other launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `params_file` | package `config/sonar3d.yaml` | Driver parameter file |
| `namespace` | `sonar3d` | Namespace for the driver and its relative topics |
| `autostart` | `true` | Configure and activate the driver on start; `false` leaves it unconfigured for a lifecycle manager |
| `container` | empty | Load the component into this running container, with intra-process communication, instead of starting `sonar_publisher` |
| `log_level` | `info` | Log level of the standalone driver process |

HTTP configuration runs on a bounded background thread, so a missing sonar
does not block the executor or receive path, and cleanup cancels requests in
flight within about a second. A sonar that is unreachable, busy or failing
(connection errors, timeouts, HTTP 408, 409, 429 and 5xx) is configured again
after 1 s, then at doubling intervals up to 30 s, until it accepts; a sonar
still booting when the driver starts is therefore configured once it is up.
A request the sonar rejects (another 4xx) is not repeated. Setting
`configure_sonar:=false` makes the node listen without changing device state.

### Sonar settings

On configure, with `configure_sonar` true, the driver applies the requested
settings through the sonar's HTTP API in dependency order, then enables
acoustics and UDP output. Every optional setting defaults to "keep the
device's value", and the sonar persists what it is given:

| Parameter | Effect | Sonar release |
| --- | --- | --- |
| `ntp_server` | NTP server for the sonar clock, an address or `auto` | 1.7.1 |
| `salinity` | `fresh` or `salt`, used by the automatic speed of sound | 1.7.0 |
| `speed_of_sound` | `0` automatic (from salinity and water temperature), or 1000–2000 m/s | all |
| `acoustics_mode` | `low-frequency` (5 Hz, 90°×40°) or `high-frequency` (20 Hz, 40°×40°); see [Acoustics modes](#acoustics-modes) | 1.7.0 |
| `range_min`, `range_max` | Imaging range in metres; applied when `range_max` > 0 | all |
| `udp_mode` | Multicast, or unicast to `interface_address`:`udp_port` | all |
| `publish_imu` | Also enables `ImuBatch` output | 1.8.0 |
| `ntp_sync_timeout` | When positive, forces an NTP sync and waits up to that many seconds | 1.7.1 |

Releases older than a setting answer 404; the driver then logs which
settings the firmware lacks and diagnostics list them in
`configuration_unsupported` as a warning, while imaging continues. Other
failures report `configuration: retrying` (or `failed` when the sonar rejected
the request) with `configuration_error` and `configuration_attempts`. A wrong speed of sound
scales every range (1491 instead of 1530 m/s is a 2.6% error), so prefer
`speed_of_sound: 0` with the right `salinity`, or a measured value. Giving the
sonar an NTP server (for example the vehicle computer) lets the driver use
its acquisition timestamps directly instead of the offset estimate described
above.

### Acoustics modes

Both modes image 256×64 pixels; they differ in field of view and rate:

| Mode | Field of view | Documented rate | Pixel spacing (azimuth × elevation) |
| --- | --- | --- | --- |
| `low-frequency` | 90° × 40° | 5 Hz (recordings run at 6.0 Hz) | 0.35° × 0.63° |
| `high-frequency` | 40° × 40° | 20 Hz | 0.16° × 0.63° |

Images carry no mode field, so the driver identifies the mode from each range
image's field of view and converts every image with its own field of view; a
mode change takes effect from the next image. It logs the mode when imaging
starts or the mode changes, and the `RIP stream` diagnostic reports
`image_fov_degrees` and `image_acoustics_mode`. The `Range image rate` bounds
follow that mode: 5–6 Hz in low frequency and 20–24 Hz in high frequency, each
with `diagnostic_updater`'s 10% tolerance, or 5–24 Hz before the first image or
for an unrecognised field of view. The upper bounds allow the 20% excess seen
in low-frequency recordings; the high-frequency rate has not yet been measured
on hardware.

Set `acoustics_mode` to the mode the vehicle should use. With
`configure_sonar` the driver applies it; either way, `RIP stream` warns when
the images show another mode, for example after the sonar's web interface
changed it. The check waits for the first configuration attempt to finish.
The `Device` diagnostic reports the mode and range the sonar itself reports.

### Unicast output

Multicast is the default. On routed networks or when multicast is filtered,
set `udp_mode: unicast` and `interface_address` to this computer's address on
the sonar's network: the driver binds `interface_address:udp_port` and, with
`configure_sonar`, tells the sonar to send there. Unicast sockets are opened
without `SO_REUSEADDR`, as in wlsonar, because Linux would deliver each
datagram to only one of several listeners; a second driver on the same port
fails to configure instead of silently stealing packets.

### Device status

While configured, the driver reads the sonar's `/about`, `/status`,
`/temperature`, `/time/status`, `/acoustics/mode` and `/acoustics/range` every
`device_status_period` seconds (5 by default; 0 disables) and publishes them as
the `Device` diagnostic: firmware version, chip ID, readiness, temperature,
each component's status (`ok`/`warning`/`error` map to OK/WARN/ERROR), NTP
sync state, `acoustics_mode`, `range_min_m` and `range_max_m`. Endpoints that
the firmware answers with 404 are left out. Once the
sonar answers, its chip ID becomes the diagnostics `hardware_id`.

### Lifecycle

The driver is a managed node:

| Transition | Effect |
| --- | --- |
| configure | Validates parameters, opens and joins the multicast socket, creates publishers, broadcasts the IMU transform, and starts HTTP configuration. Invalid parameters or a socket error fail the transition and leave the node unconfigured. |
| activate | Discards datagrams queued while inactive, resets sequence tracking, and starts streaming. |
| deactivate | Stops streaming; the socket and device settings stay in place. |
| cleanup | Releases the socket and publishers and stops pending HTTP configuration. |

Driver parameters can be changed only while the node is unconfigured and take
effect on the next configure, so a running driver never reports a value it is
not using:

```bash
ros2 lifecycle set /sonar3d/sonar3d_driver deactivate
ros2 lifecycle set /sonar3d/sonar3d_driver cleanup
ros2 param set /sonar3d/sonar3d_driver frame_id sonar_front_link
ros2 lifecycle set /sonar3d/sonar3d_driver configure
ros2 lifecycle set /sonar3d/sonar3d_driver activate
```

The launch file sets the driver's `autostart` parameter, which configures and
activates it as soon as it spins, both standalone and in a container. The
node's own default is `false`, as a lifecycle manager such as
`nav2_lifecycle_manager` expects; run it directly with
`ros2 run sonar3d sonar_publisher --ros-args -p autostart:=true`, or drive the
transitions with `ros2 lifecycle set`. The component can also be loaded into
an existing container directly:

```bash
ros2 component load /ComponentManager sonar3d sonar3d::SonarDriver -p autostart:=true
```

`/diagnostics` is published in every state. The `RIP stream` status reports
the lifecycle state and is OK with "not streaming" while inactive; the
`Range image rate` status (bounds per [acoustics mode](#acoustics-modes)) is added while active, and the
`Device` status is filled while configured. The update period is
`diagnostic_updater`'s standard `diagnostic_updater.period` parameter.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `autostart` | `false` | Configure and activate when the node starts spinning (the launch file sets `true`) |
| `sonar_ip` | `192.168.194.96` | Literal IPv4 packet source and HTTP API address; empty accepts any source when configuration is disabled |
| `fallback_ip` | `192.168.194.96` | Additional accepted packet source, by default the sonar's fixed fallback address; empty accepts `sonar_ip` only |
| `frame_id` | `sonar3d_link` | Frame for range, bitmap, and point-cloud products |
| `imu_frame_id` | `sonar3d_imu_link` | Frame at the internal IMU origin |
| `configure_sonar` | `true` | On configure, apply the sonar settings above and enable acoustics, UDP output and (with `publish_imu`) IMU output, retrying while the sonar is unreachable or busy |
| `speed_of_sound` | `-1.0` | Negative keeps the device setting; `0` automatic; otherwise 1000–2000 m/s |
| `salinity` | empty | `fresh` or `salt`; empty keeps the device setting |
| `acoustics_mode` | empty | `low-frequency` or `high-frequency`; empty keeps the device setting. Diagnostics warn when images show another mode, also with `configure_sonar` false |
| `range_min` | `0.0` | Minimum imaging range in metres, applied with `range_max` |
| `range_max` | `0.0` | Maximum imaging range in metres; zero keeps the device range |
| `ntp_server` | empty | NTP server address or `auto`; empty keeps the device setting |
| `ntp_sync_timeout` | `0.0` | Positive: force an NTP sync during configure and wait up to this long |
| `http_port` | `80` | Sonar HTTP API port |
| `http_timeout` | `5.0` | Ordinary HTTP timeout in seconds; acoustics settings are allowed at least 30 s |
| `device_status_period` | `5.0` | Seconds between device status reads; zero disables |
| `udp_mode` | `multicast` | `multicast` or `unicast` |
| `multicast_group` | `224.0.0.96` | RIP multicast group (multicast mode) |
| `udp_port` | `4747` | RIP UDP port |
| `interface_address` | `0.0.0.0` | Multicast: interface joining the group. Unicast: local address to bind and, with `configure_sonar`, the sonar's destination |
| `udp_receive_buffer_size` | `1048576` | Requested socket receive buffer in bytes; zero keeps the OS default |
| `use_sensor_timestamps` | `true` | Prefer valid device timestamps over receive time |
| `max_sensor_clock_offset` | `1.0` | Maximum absolute minimum-observed clock offset in seconds before sensor timestamps are shifted onto ROS time; zero trusts the sensor clock unconditionally |
| `publish_tf` | `true` | Broadcast the static `frame_id` to `imu_frame_id` transform |
| `point_cloud_intensity` | `true` | Add linear signal strength from the paired signal image as `intensity` |
| `publish_point_cloud` | `true` | Enable the derived cloud |
| `publish_raw_packets` | `true` | Enable exact RIP packet publication on demand |
| `publish_range_image` | `true` | Enable the scaled range image |
| `publish_bitmap_images` | `true` | Enable signal-strength and shaded bitmap topics |
| `publish_imu` | `true` | Publish `ImuBatch` samples on `imu/data_raw` |
| `publish_vendor_orientation` | `false` | Experimental: decode existing private `ImuOrientation` output into orientation-only `imu/data`; does not enable private output on the device |
| `linear_acceleration_stddev` | `0.0` | REP-145 accelerometer noise in m/s²; zero reports the covariance as unknown |
| `angular_velocity_stddev` | `0.0` | REP-145 gyroscope noise in rad/s; zero reports the covariance as unknown |
| `packet_stale_timeout` | `2.0` | Time without valid packets before diagnostics report a stale stream |
| `diagnostic_updater.period` | `1.0` | `/diagnostics` period in seconds (owned by `diagnostic_updater`) |

`fallback_ip` carries Water Linked's fix from the original Python driver: on
some vehicles (Blueye, for example) the sonar's UDP output arrives from its
fixed fallback address even when it holds a DHCP address. Packets from any
other address are counted as `rejected_sources` and logged once per address.

A dedicated receive thread blocks on the socket and publishes each product as
soon as its datagram arrives, independent of executor load, and sleeps while
the sonar is silent. The receive buffer holds packet bursts while that thread
is converting. The OS can adjust or clamp the request; `/diagnostics` reports
the effective socket value as `udp_receive_buffer_bytes`. This setting
changes only the driver's socket.

## Native recording during live operation

With the driver active, record an MCAP bag and a native `.sonar` file together:

```bash
ros2 run sonar3d sonar_record_bag --output /absolute/path/to/new_dive
# Ctrl+C pauses the bag, drains/fsyncs the native writer, then finalizes the bag.
```

The parent directory must exist; the output directory must be new. The command
creates `new_dive/sonar.sonar` and `new_dive/native_recording.json` alongside
the bag. The manifest reports final packet/byte counts and any capture error.
The default bag topics are the driver's raw packets, images, points, metadata,
public raw IMU, `/tf_static`, and `/diagnostics`. Add other vehicle topics with
standard bag arguments after `--`, for example:

```bash
ros2 run sonar3d sonar_record_bag --output /data/new_dive \
  --driver-node /sonar3d/sonar3d_driver -- \
  --topics /sonar3d/raw_packet /camera/image_raw /tf /tf_static /odom /diagnostics
```

Use `--duration 60` for a timed capture. `--driver-node` identifies the fully
qualified lifecycle node; its default is `/sonar3d/sonar3d_driver`. The helper
starts the bag paused, starts native capture, then resumes the bag. On exit it
pauses the bag before stopping native capture. Native capture therefore has
small pre/post-roll around the bag, rather than exact timestamp boundaries.
Wait for `RECORDING READY` before beginning a test. Pausing the bag separately
does not pause native capture; use this helper's Ctrl+C to stop both.

Native capture does not require ROS topic subscribers or `publish_raw_packets`.
It stores accepted-source UDP datagrams before decoding and sequence checks,
preserving unknown fields, duplicates, and damaged packets. The format is the
same concatenation of RIP datagrams used by the reference `.sonar` files;
no device-side recording API or reconstruction of processed clouds is involved.
It captures only products the sonar actually transmits. Packet loss before the
host receives a datagram cannot be recovered by this writer.

The receive thread enqueues data to a disk worker. The default bounded queue is
64 MiB (`native_recording_queue_bytes`), plus one in-flight datagram. Queue
overflow or disk errors stop capture, persist an error in the services and
`Native recording` diagnostic, and cause the helper to exit unsuccessfully.
Existing files are never overwritten. Deactivation/shutdown drains and closes
an active capture. A forced process kill or power loss cannot guarantee a flush.

For another bag orchestration script, call the driver's services directly:

```bash
ros2 service call /sonar3d/sonar3d_driver/start_recording sonar3d/srv/StartRecording \
  '{path: /absolute/path/to/dive.sonar}'
ros2 service call /sonar3d/sonar3d_driver/get_recording_status sonar3d/srv/GetRecordingStatus '{}'
ros2 service call /sonar3d/sonar3d_driver/stop_recording sonar3d/srv/StopRecording '{}'
```

Starting requires an active driver and an absolute `.sonar` path whose parent
exists. Stop waits for accepted packets to be written and fsynced. Check
`success` and `status.error`; an incomplete recording remains an error until a
new capture starts. Repeated stops are safe. Ordinary `ros2 bag record` alone
does not invoke these services.

## Recording playback

Playback is native C++:

```bash
ros2 run sonar3d sonar_replay \
  --file survey.sonar \
  --realtime-factor 1.0 \
  --frame-id sonar3d_link \
  --ros-args -r __ns:=/sonar3d
```

The player reads mixed RIP1/RIP2 recordings using declared packet lengths,
validates CRCs, and skips damaged packets when framing remains recoverable. It
pairs range and signal images for the cloud's `intensity` like the live driver
(`--no-intensity` omits the field) and reports range images without a signal
image. The old `sonar_to_bag` executable name remains as an alias.

### Converting a recording to a bag

```bash
ros2 run sonar3d sonar_to_bag --file survey.sonar --output survey_bag \
  --ros-args -r __ns:=/sonar3d
```

`--output` writes every supported product straight into a new rosbag2 bag
(MCAP by default; choose another plugin with `--storage sqlite3`) instead of
publishing. There is no DDS discovery, no `ros2 bag record` process, and no
real-time pacing, so conversion runs as fast as decoding and loses nothing.
Each message is stored at its recorded sensor timestamp, which is also its
header stamp; `--receive-time` is rejected in this mode. Topic names follow
the node namespace, so the example writes `/sonar3d/points`,
`/sonar3d/imu/data_raw` and so on, matching the live driver. The summary lists
the messages written per topic, and framing errors fail the conversion as in
playback. The bag also includes exact `raw_packet` bytes, including complete
damaged and unsupported packets. Recordings lack sender addresses and host
receipt times, so their raw headers use the packet's acquisition reference,
or the previous/first known reference when its type cannot be interpreted.

### Publishing a recording

Without `--output`, the player publishes the same data topics as the live
driver, reliably with a 100-message history. A short startup
delay allows DDS discovery before the first sample; control it with
`--startup-delay`. Use `--receive-time` to ignore recorded sensor timestamps.

Playback uses deadlines anchored to the first valid recorded timestamp, so
decoding and publication consume the recorded interval instead of extending it.
Repeated timestamps and older interleaved IMU samples are sent without an extra
delay. A backward clock jump exceeding one second starts a new timing segment.
`--realtime-factor` scales these intervals; `--receive-time` changes ROS message
stamps while retaining recorded pacing. Start subscribers (including rosbag)
before playback and allow enough startup delay for discovery.

During paced playback, a range image missing its signal image is released
after the same 25 ms wall-clock wait as live acquisition. Offline bag
conversion pairs by recorded shot without a wall-clock deadline.

The summary separates supported packets, unsupported message types, damaged
packets (CRC, decompression, or protobuf envelope errors), malformed supported
products, and lost framing. Each unsupported type is listed with its count;
each published topic is listed with its ROS message count. A processed packet
does not imply publication or full conversion validation: normal playback
converts only products with subscribers.

Validate all supported image, point-cloud, and public IMU conversions without
subscribers or recorded delays:

```bash
ros2 run sonar3d sonar_replay --file survey.sonar --validate-only
```

This mode creates no data publishers, ignores startup delay and playback pacing,
and returns a nonzero status for damaged packets, malformed supported products,
or lost framing. Unsupported types are counted separately and do not fail
validation. Interrupted validation also returns a nonzero status.
For unsupported types, only packet framing, CRC, and protobuf envelopes are checked.
Normal playback continues past recoverable damaged/malformed packets and fails
if framing is lost.

Some GUI recordings contain private `waterlinked.sonar.internal.ImuOrientation`
and `ImuRaw` messages. `ImuOrientation` is decoded and validated; publication
requires `--vendor-orientation` (see below). `ImuRaw` remains unsupported.
Neither private type is substituted for the public `ImuBatch` on `imu/data_raw`.
Keep the original recordings to retain private telemetry. For future recordings,
enable public raw IMU output before recording. The vendor manual (Integration
API, pages 36–38) describes 100 Hz `ImuBatch` output, disabled by default, and
its linked
[HTTP specification](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api-swagger/swagger.json)
defines a boolean POST to `/api/v1/integration/output/imu-batch/enabled`.

With `configure_sonar` and `publish_imu` both true (the defaults), the live
driver sends that request itself after enabling acoustics and multicast.
Releases before 1.8.0 answer 404; the driver then logs that IMU output is
unsupported and diagnostics report `configuration_unsupported: imu_output`
as a warning, while imaging continues. With `configure_sonar:=false`, enable
it yourself:

```bash
SONAR_IP=192.168.194.96  # Set this to your sonar's address.
curl --fail --show-error \
  -X POST -H 'Content-Type: application/json' --data 'true' \
  "http://${SONAR_IP}/api/v1/integration/output/imu-batch/enabled"
```

Existing recordings need recorded public `ImuBatch` packets to replay samples
on `imu/data_raw`; enabling device output later cannot add them.

### Vendor orientation (experimental)

The public [vendor integration protocol](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/)
does not define the private orientation schema. The public vendor replay app
also does not decode it. The definition in
`proto/WaterLinkedSonarInternalProtocol.proto` is inferred from recordings,
not a vendor-supported API: field 1 is the public `Header`, fields 2 and 3
are retained as opaque uint32 values, and field 4 is four packed float32
quaternion coefficients. Unknown additional fields and the original datagram
remain recoverable through `raw_packet`.

The 2026-01-29 recording was used to distinguish quaternion conventions by
comparing its private raw IMU quaternion samples (field 8) to angular rates
(field 7) and specific force (field 6). Scalar-first **w,x,y,z**, body-to-world
rotation gave gyro correlations of 0.999, 0.985 and 0.989 across the three axes.
Rotated specific force had mean (-0.008, -0.020, -9.719) m/s² with standard
deviations (0.147, 0.120, 0.206); inverse and scalar-last interpretations did
not satisfy both checks. This is empirical evidence, not a calibrated
attitude or heading reference. The HF packets match this wire layout and
their quaternion norms are checked against unity before use.
The check used 12,030 raw samples; 716 of 721 orientation packets have
coefficients present byte-for-byte in those samples. Source SHA-256 hashes,
alternative convention scores and boundary-packet audits are retained in
[the research evidence](docs/vendor_orientation_evidence.json), and
[full HF replay results](docs/hf_orientation_replay_results.json) verify every
orientation/cloud count and the exact cloud-to-TF timestamp matches over DDS.

Publication is opt-in:

```bash
ros2 run sonar3d sonar_replay --file survey.sonar --vendor-orientation \
  --ros-args -r __ns:=/sonar3d
# Live: only works when the device already transmits this private product.
ros2 launch sonar3d sonar3d.launch.py publish_vendor_orientation:=true
```

`imu/data` carries only orientation at the source shot timestamp, in
`imu_frame_id`. Both vendor body and reference bases are changed from z-down
to z-up: `R_ros = C R_vendor C^-1`, where `C = diag(1,-1,-1)`; equivalently
`(w,x,y,z)` becomes `(w,x,-y,-z)`. Near-unit quaternions are normalized;
nonfinite, wrong-sized or non-unit estimates are rejected before sequence
tracking or clock estimation. Orientation covariance is unknown (all zeros);
angular velocity and acceleration are absent (their covariance[0] is -1).
There is no claim of ENU alignment, north-referenced yaw or position.

For an attitude-only RViz preview:

```bash
ros2 launch sonar3d sonar_replay.launch.py file:=/absolute/path/survey.sonar \
  vendor_orientation:=true image_range_max:=5.0
```

This uses `sonar3d_attitude` as a local z-up fixed frame and publishes a
rotation-only transform to `frame_id`, with zero translation. Each cloud TF
uses the orientation with **the same source sequence and timestamp**, stamped
at that image's ROS time, so delayed signal pairing cannot misalign the TF.
An image without that exact orientation keeps its raw image products but is
omitted from the attitude-only cloud preview and reported in the summary.
Without the preview option all measured clouds are retained. The regular live
driver still publishes only its documented static sonar-to-IMU transform.

The four November HF recordings contain 256×64 images over 40°×40° at
19.67–19.72 Hz. Each wheel clip ends after a range packet whose signal packet
is absent: the final cloud is preserved with intensity zero, without copying
another shot's intensity. `167_wheel` also starts with a range/signal pair
whose orientation is absent. These are recording-boundary omissions, not CRC
or decoding failures, and the missing measurements cannot be reconstructed.

### Standalone RViz playback

```bash
ros2 launch sonar3d sonar_replay.launch.py file:=/absolute/path/survey.sonar
```

The included layout shows the current cloud and signal-strength image on the
driver's topic names in the `sonar3d` namespace. Enable the optional Shaded depth display for
recordings containing shaded images. Its fixed frame is `sonar3d_link`,
so no vehicle model or identity `odom` transform is needed. The viewer publishes
the documented sensor-to-IMU static transform to provide a complete sensor TF
tree and avoid RViz's missing-frame warning (the live driver broadcasts the
same transform itself). Message stamps use
the current ROS clock while pacing follows the recording. The default view is
in the sonar frame; `vendor_orientation:=true` enables the experimental
attitude-only preview described above.

Optional launch arguments are `realtime_factor`, `startup_delay`, `frame_id`,
`imu_frame_id`, `namespace`, `vendor_orientation`, `orientation_frame`, and
`rviz` (`false` for headless replay). Closing RViz stops replay;
after recording EOF, RViz stays open with the last frame.
Set `loop:=true` to restart playback after EOF while keeping RViz open.

In the Nautilus workspace, the host helper stages a recording into the shared
`/tmp` mount and passes the current display credentials to the GPU container:

```bash
./scripts/sonar3d_replay_gpu.sh '/home/jcfurey/Downloads/recording (1).sonar'
```

Run it from the host graphical session with `DISPLAY` and `XAUTHORITY` set.
It uses the existing Xauthority cookie rather than changing X-server access
control. `SONAR3D_CONTAINER` and `SONAR3D_CONTAINER_WS` override the default
Nautilus container name and workspace path. Press Ctrl+C to stop the launch
and remove its staged copy.

Record a live replay with standard rosbag tooling (or convert the file
directly with `--output` as above):

```bash
ros2 bag record \
  /sonar3d/raw_packet \
  /sonar3d/range_image \
  /sonar3d/intensity_image \
  /sonar3d/shaded_image \
  /sonar3d/points \
  /sonar3d/imu/data_raw \
  /sonar3d/image_metadata \
  /tf_static
```

### Quantitative 2D images

The raw image topics remain measurement data. `image_metadata` supplies the
settings for each image, paired by **image type, frame, and exact ROS header
timestamp**. It retains the original sonar timestamp separately from the ROS
header: receive-time playback and live clock re-anchoring do not erase the
recorded clock. A sonar clock reading is not evidence of NTP synchronization.
`frequency` is copied as the vendor field without assigning an undocumented
unit. `driver_source_sha256` fingerprints the native production source,
headers, protocol/interface definitions and build/package descriptions.

The optional viewer uses the same rendering code for live viewing and saved
figures:

```bash
ros2 run sonar3d sonar_image_viewer --namespace sonar3d \
  --range-limits 0 15 --signal-limits 0 255 --output image-captures
```

Or add it to standalone replay, including the Nautilus GPU helper:

```bash
ros2 launch sonar3d sonar_replay.launch.py file:=/absolute/path/survey.sonar \
  scientific_images:=true image_range_min:=0 image_range_max:=15
```

RViz replay now shows the same labelled figure in its **Quantitative images**
panel automatically, using a headless `sonar_image_publisher`. The RGB image
includes angular axes, metre/code colour bars, source and ROS clocks, coverage,
and acquisition settings. It uses the same `ScientificFigure` renderer as the
offline figures. This is a display-only raster; use `range_image` and
`intensity_image` for analysis, not the plotted pixels.

The default display rate is 3 Hz, selecting the latest ready shot without
averaging; raw measurements keep their original publication rate. The output
uses reliable, transient-local QoS with depth 1, allowing a newly enabled RViz
panel to receive the last plotted frame after replay ends. Rendering pauses
when there are no output subscribers. `rviz_images:=false` disables the replay
renderer. Raw range, raw signal and qualitative shaded displays remain optional;
the raw range panel explicitly disables RViz's per-frame normalization and
uses 0–15 m. Adjust these raw-panel limits manually for other configured ranges.

For a live driver, enable `quantitative_images:=true` in `sonar3d.launch.py`,
or start the renderer alongside an existing driver:

```bash
ros2 run sonar3d sonar_image_publisher --namespace sonar3d \
  --range-limits 0 15 --signal-limits 0 255 --refresh-hz 3
```

Add an RViz Image display on `/sonar3d/quantitative_image/compressed`, with
Reliable and Transient Local QoS. Both bundled RViz configurations
select the lossless PNG `quantitative_image/compressed` topic. Raw RGB remains
available for subscribers requesting it; each requested figure is drawn once,
and encoding/publication runs only for subscribed transports. RGB colours are already fixed by the renderer; RViz's
floating-point normalization does not apply. See the
[Jazzy Image display implementation](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/image/image_display.cpp).

Closing either enabled viewer stops the launch. The image viewer offers
**Pause/Resume** and **Save frame**; pausing freezes the displayed shot while
reception continues. It displays only products with matching source shot
IDs/clocks and geometry, never a signal image from the previous shot. If a
signal image is missing, the range remains viewable with signal marked
unavailable.

The plots show **slant range in metres** and the vendor's logarithmic signal
code. Colour limits are fixed (defaults 0–15 m and 0–255), never normalized to
each frame. Zero/no-return pixels are grey and excluded from summaries;
out-of-scale returns use distinct under/over colours and are counted. The
[perceptually uniform sequential colormap](https://matplotlib.org/stable/users/explain/colors/colormaps.html)
defaults to `cividis`; `viridis` and `magma` are also available. Angular axes use
REP-103 degrees, positive left and up. Vendor first/last pixel **centres** lie
at the FOV endpoints; figure boundaries extend half a bin beyond these centres.
Pixels are not interpolated. Image cells represent the strongest return in
each direction, not a Cartesian slice or a range-time echogram.

For vendor-relative linear signal strength, use `--signal-scale linear`
(default limits 0–10644). Its documented conversion is
`round(30 * 10 ** (code / 100))`, with zero retained as no return. This is
**not calibrated acoustic backscatter**. Shaded bitmaps are saved as qualitative
vendor codes when present; metric distance always comes from `range_image`.

Export a selected frame reproducibly from an original recording:

```bash
ros2 run sonar3d sonar_image_viewer --file survey.sonar --frame-index 0 \
  --range-limits 0 15 --signal-limits 0 255 --output figures/shot-0
```

The native player first validates supported products, then converts to a temporary bag with
metadata, then the viewer reads the selected shot. `--frame-index` is zero
based in bag read order; `--sequence-id` selects a source shot instead.
`--bag` reads an existing rosbag2 bag containing `image_metadata`. Recordings
or older bags without metadata cannot provide angular/acquisition provenance;
use the original `.sonar` input to generate it.

Each new bundle contains `figure.png`, `figure.pdf`, `data.npz`, and
`metadata.json`. Arrays preserve upright float32 metre measurements and
uint8 signal/shaded codes, with valid-return masks, relative linear strength
and angular pixel centres. The manifest records source SHA-256, native
converter/library and driver source fingerprints, exact integer timestamps,
shot/settings metadata, colour limits/units, array checksums, and Python,
NumPy and Matplotlib versions. Raw ROS range precision is retained; keep
the original recording for its pre-conversion integer pixels and telemetry.
Existing bundles are never overwritten.

Verify and rerender a bundle with its saved colour settings:

```bash
ros2 run sonar3d sonar_image_viewer --bundle figures/shot-0 --output figures/shot-0-copy
```

Array checksums are verified before rendering. Fixed data, settings and the
same plotting environment reproduce the PNG; font/backend/library version
changes can change rendering, which is why the numeric data and environment
are retained. Display scale changes do not modify saved measurements. Standalone
replay also attaches the recording's SHA-256 to live snapshots; when starting
the image viewer separately for replay, pass `--source-recording survey.sonar`.

## Tests

Native capture tests cover exact-byte preservation, exclusive file creation,
queue overflow, restart, destructor/deactivation flushing, and capture with all
ROS data products disabled. To additionally round-trip vendor recordings:

```bash
SONAR3D_REFERENCE_DIR="$HOME/Downloads" colcon test --packages-select sonar3d \
  --ctest-args -R test_native_recorder --output-on-failure
```

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
Replay regressions also check subscriber-free validation, unsupported private
telemetry counts, CRC failures, malformed image/IMU products, per-topic
publication counts, fatal framing summaries, and clean interruption with an
explicit incomplete-results warning.
Deterministic clock tests cover processing time, interleaved IMU batches, speed
factors, and recorded clock resets. Live stream tests check that an
unsynchronized sonar clock is aligned to receive time with IMU spacing
preserved, that one shared offset keeps the sonar's spacing between images and
IMU samples delivered at different delays, that a synchronized clock keeps
exact sensor timestamps, and that the fallback source is accepted while other
sources are rejected. Offset-estimator tests cover latency spikes, drift, and
clock steps. Image tests check the upright row order against the vendor's
pixel-to-angle formula, and bag tests check that `--output` writes every
product with its sensor timestamp without pacing. Lifecycle tests check that
deactivation pauses streaming and discards stale datagrams without counting
false sequence gaps, that parameters change only while unconfigured and apply
on the next configure, that invalid parameters fail configure recoverably, and
that the `autostart` parameter activates the node. A reliable subscriber test
guards REP-2003 compatibility, HTTP tests cover the IMU output step and its
404 fallback, and IMU tests cover REP-145 covariances. Sonar API tests run
the real HTTP client against an in-process server to check the JSON body of
every setting, status parsing, 404 handling for older firmware, and
cancellation of a hung request; a driver test configures a stand-in sonar end
to end and checks the `Device` diagnostic and its chip-ID `hardware_id`.
Pairing tests cover either arrival order, lost and stale signal images, and
sequence wraparound; stream tests cover unicast delivery, unpaired release
after the wait, and the cloud without intensity. Acoustics mode tests check
that the field of view identifies each mode, that the rate bounds follow it,
that images from an unexpected mode warn, that the `Device` diagnostic reports
the sonar's mode and range, and that configuration is retried after a
transient failure but not after the sonar rejects a request.

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
transport (`dds` or `ipc`). The `all` selection includes raw packets and image
metadata as well as processed products. It simulates one sonar over loopback multicast with
256×64 images and 100 Hz RIP2 IMU samples. It reports delivered counts, sequence
gaps, range/cloud latency, and CPU for the entire test process, including the
sender and subscriber. HTTP configuration is disabled. Nonzero exit status
indicates incomplete delivery; timing values are measurements, not pass limits.
Use an unused `ROS_DOMAIN_ID` to isolate these ROS topics from other tests.

## Protocol source and license

The wire definition and behavior follow Water Linked's
[Sonar 3D-15 integration API](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/)
its [HTTP specification](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api-swagger/swagger.json),
and [`wlsonar` 0.5.5](https://github.com/waterlinked/wlsonar/tree/v0.5.5), whose
protocol definition is unchanged since 0.5.4.
Both the driver and vendored protocol definition are MIT licensed. Recording
playback history retains attribution to Marios Xanthidis (SINTEF Ocean), the
Research Council of Norway EchoNav project, and Alberto Quattrini Li's earlier
ROS integration.
