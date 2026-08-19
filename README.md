# Sonar 3D-15 ROS Driver

ROS 2 driver and recording playback tools for the Water Linked Sonar 3D-15.
The package is tested with ROS 2 Jazzy. It receives RIP1 and RIP2 UDP packets,
decodes them with Water Linked's official
[`wlsonar`](https://github.com/waterlinked/wlsonar) library, and publishes the
range, intensity, and point-cloud products without requiring the browser
interface.

## ROS topics

The live driver and recording player use the same relative topic names, so a
platform launch file can namespace or remap them normally.

| Topic | Type | Contract |
| --- | --- | --- |
| `sonar_range_image` | `sensor_msgs/Image` | `32FC1` range in meters; zero is no return |
| `sonar_intensity_image` | `sensor_msgs/Image` | Native, unscaled `mono8` vendor bitmap |
| `sonar_point_cloud` | `sensor_msgs/PointCloud2` | Dense valid returns with `x,y,z,range,azimuth,elevation` float32 fields |

The point cloud uses a ROS REP-103 body frame: x forward, y left, z up. The
vendor's y-right/z-down coordinates and native yaw sign are converted when the
cloud is formed. Images retain the vendor's row-major pixel ordering. Sensor
timestamps are used when present; the ROS clock is only a fallback.

## Dependencies and build

`wlsonar` is distributed on PyPI and does not currently have a rosdep key. The
package pins the validated release in both `requirements.txt` and `setup.py`.
Install it on the target platform before building:

```bash
python3 -m pip install -r src/sonar3d/requirements.txt
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sonar3d
source install/setup.bash
```

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
| `speed_of_sound` | `0.0` | m/s value to configure; zero preserves the sonar setting |
| `configure_sonar` | `true` | Enable acoustics and multicast through the HTTP API |
| `http_timeout` | `5.0` | Maximum seconds for ordinary HTTP API requests |
| `multicast_group` | `224.0.0.96` | UDP multicast group |
| `multicast_port` | `4747` | UDP multicast port |
| `multicast_interface` | `0.0.0.0` | Local IPv4 interface used to join multicast |
| `poll_period` | `0.01` | ROS timer period in seconds |
| `max_packets_per_spin` | `32` | Maximum datagrams drained per timer callback |

The receive socket is nonblocking, so the ROS executor and shutdown path remain
responsive when the sonar is absent or stops transmitting. HTTP setup calls are
also bounded; setting speed of sound permits up to 30 seconds because the device
may apply that setting slowly. Set `configure_sonar:=false` to listen to a sonar
that is already configured without making HTTP requests.

## Recording playback

The installed playback executable reads mixed RIP1/RIP2 `.sonar` recordings and
publishes the same topic contract as the live driver:

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

Playback follows packet lengths and validates CRCs. A damaged packet with valid
framing is skipped; playback stops if stream framing is lost. RIP1 and RIP2 are
both decoded by the same official parser.

## License and attribution

The ROS package is MIT licensed. Recording playback work includes attribution to
Marios Xanthidis (SINTEF Ocean), the Research Council of Norway EchoNav project,
and Alberto Quattrini Li's earlier ROS integration, as described in the source
history.
