# Sonar 3D-15 ROS Driver

ROS 2 driver and recording playback tools for the Water Linked Sonar 3D-15.
The package is tested with ROS 2 Jazzy. It receives RIP1 UDP multicast packets,
decodes the vendor protobuf messages, and publishes the range, intensity, and
point-cloud products without requiring the browser interface.

The protocol definitions are pinned as a nested git submodule. Clone this
repository with `--recurse-submodules`, or initialize it afterward with:

```bash
git submodule update --init --recursive
```

## ROS topics

All topic names are relative, so a platform launch file can namespace or remap
them normally.

| Topic | Type | Contract |
| --- | --- | --- |
| `sonar_range_image` | `sensor_msgs/Image` | `32FC1` range in meters; zero is no return |
| `sonar_intensity_image` | `sensor_msgs/Image` | Native, unscaled `mono8` vendor bitmap |
| `sonar_point_cloud` | `sensor_msgs/PointCloud2` | Dense valid returns with `x,y,z,range,azimuth,elevation` float32 fields |

The point cloud uses a ROS REP-103 body frame: x forward, y left, z up. The
vendor's y-right/z-down coordinates and native yaw sign are converted when the
cloud is formed. Images retain the vendor's row-major pixel ordering. Sensor
timestamps are used when present; the ROS clock is only a fallback.

## Build

Install dependencies through the package manifest and build from the workspace
root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sonar3d
source install/setup.bash
```

For non-ROS protocol tooling, Python dependencies are also listed in
`src/sonar3d/requirements.txt`.

## Live operation

```bash
ros2 launch sonar3d sonar3d.launch.py \
  sonar_ip:=192.168.194.96 \
  frame_id:=sonar3d_link \
  multicast_interface:=0.0.0.0
```

Launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `sonar_ip` | `192.168.194.96` | Expected packet source and HTTP API address |
| `frame_id` | `sonar3d_link` | Frame attached to all published products |
| `speed_of_sound` | `0` | m/s value to configure; zero preserves the sonar setting |
| `configure_sonar` | `true` | Enable acoustics and multicast through the HTTP API |
| `http_timeout` | `5.0` | Maximum seconds for ordinary HTTP API requests |
| `multicast_interface` | `0.0.0.0` | Local IPv4 interface used to join multicast |

The receive socket is nonblocking, so the ROS executor and shutdown path remain
responsive when the sonar is absent or stops transmitting. HTTP setup calls are
also bounded; setting speed of sound permits up to 30 seconds because the device
may apply that setting slowly.

## Recording playback

The installed playback executable publishes the same topic contract as the live
driver:

```bash
ros2 run sonar3d sonar_to_bag \
  --file survey.sonar \
  --realtime-factor 1.0 \
  --frame-id sonar3d_link
```

Record the replay with normal rosbag tooling if an MCAP or SQLite bag is needed:

```bash
ros2 bag record \
  /sonar_range_image \
  /sonar_intensity_image \
  /sonar_point_cloud
```

The playback reader walks RIP1 packet lengths rather than splitting blindly on
the magic bytes, so an incidental `RIP1` sequence inside a protobuf payload does
not corrupt packet boundaries.

## License and attribution

The ROS package is MIT licensed. Recording playback work includes attribution to
Marios Xanthidis (SINTEF Ocean), the Research Council of Norway EchoNav project,
and Alberto Quattrini Li's earlier ROS integration, as described in the source
history.
