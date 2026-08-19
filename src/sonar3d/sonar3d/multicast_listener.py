"""ROS 2 live driver for Water Linked Sonar 3D-15 multicast data."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from sonar3d.http_api import configure_sonar
from sonar3d.ros_messages import (
    bitmap_image_message,
    header_from_sonar_message,
    point_cloud_message,
    range_image_message,
)
import wlsonar
import wlsonar.range_image_protocol as rip


class Sonar3DNode(Node):
    """Receive RIP1/RIP2 datagrams and publish native ROS products."""

    def __init__(self):
        """Configure the sonar, multicast socket, and ROS publishers."""
        super().__init__('sonar3d_driver')

        self.declare_parameter('sonar_ip', wlsonar.FALLBACK_IP)
        self.declare_parameter('frame_id', 'sonar3d_link')
        self.declare_parameter('speed_of_sound', 0.0)
        self.declare_parameter('configure_sonar', True)
        self.declare_parameter('http_timeout', 5.0)
        self.declare_parameter('multicast_group', wlsonar.DEFAULT_MCAST_GRP)
        self.declare_parameter('multicast_port', wlsonar.DEFAULT_MCAST_PORT)
        self.declare_parameter('multicast_interface', '0.0.0.0')
        self.declare_parameter('poll_period', 0.01)
        self.declare_parameter('max_packets_per_spin', 32)

        self.sonar_ip = str(self.get_parameter('sonar_ip').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.max_packets_per_spin = int(
            self.get_parameter('max_packets_per_spin').value
        )
        if self.max_packets_per_spin <= 0:
            raise ValueError('max_packets_per_spin must be greater than zero')
        self._warned_sources = set()

        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            'sonar_point_cloud',
            qos_profile_sensor_data,
        )
        self.range_image_publisher = self.create_publisher(
            Image,
            'sonar_range_image',
            qos_profile_sensor_data,
        )
        self.intensity_image_publisher = self.create_publisher(
            Image,
            'sonar_intensity_image',
            qos_profile_sensor_data,
        )

        if self.get_parameter('configure_sonar').value:
            self._configure_sonar()

        multicast_group = str(self.get_parameter('multicast_group').value)
        multicast_port = int(self.get_parameter('multicast_port').value)
        multicast_interface = str(
            self.get_parameter('multicast_interface').value
        )
        self.sock = self._open_multicast_socket(
            multicast_group,
            multicast_port,
            multicast_interface,
        )
        poll_period = float(self.get_parameter('poll_period').value)
        if poll_period <= 0.0:
            self.sock.close()
            raise ValueError('poll_period must be greater than zero')
        self.timer = self.create_timer(poll_period, self._poll_socket)
        self.get_logger().info(
            f'Listening for Sonar 3D-15 RIP1/RIP2 packets on '
            f'{multicast_group}:{multicast_port}'
        )

    @staticmethod
    def _open_multicast_socket(group, port, interface):
        """Create a nonblocking UDP multicast receiver with wlsonar."""
        sock = wlsonar.open_sonar_udp_multicast_socket(
            mcast_group=group,
            udp_port=port,
            iface_ip=interface,
        )
        sock.setblocking(False)
        return sock

    def _configure_sonar(self):
        """Apply requested HTTP settings before listening for datagrams."""
        speed_of_sound = float(self.get_parameter('speed_of_sound').value)
        http_timeout = float(self.get_parameter('http_timeout').value)
        try:
            applied = configure_sonar(
                self.sonar_ip,
                speed_of_sound=speed_of_sound,
                timeout=http_timeout,
            )
            self.get_logger().info(f'Configured sonar settings: {", ".join(applied)}')
        except Exception as error:
            self.get_logger().warning(
                f'Could not configure Sonar 3D-15 at {self.sonar_ip}: {error}'
            )

    def _poll_socket(self):
        """Drain a bounded batch without blocking the ROS executor."""
        for _ in range(self.max_packets_per_spin):
            try:
                data, address = self.sock.recvfrom(wlsonar.UDP_MAX_DATAGRAM_SIZE)
            except BlockingIOError:
                return
            except OSError as error:
                if rclpy.ok():
                    self.get_logger().error(f'Multicast receive failed: {error}')
                return
            self._handle_datagram(data, address[0])

    def _handle_datagram(self, data, source_ip):
        """Decode and publish one datagram if it came from the selected sonar."""
        if self.sonar_ip and source_ip != self.sonar_ip:
            if source_ip not in self._warned_sources:
                self._warned_sources.add(source_ip)
                self.get_logger().warning(
                    f'Ignoring packets from {source_ip}; expected {self.sonar_ip}'
                )
            return

        try:
            message = rip.unpackb(data)
        except rip.UnknownProtobufTypeError as error:
            self.get_logger().debug(str(error))
            return
        except (
                rip.BadIDError,
                rip.CRCMismatchError,
                rip.ExtraDataError,
                ValueError,
        ) as error:
            self.get_logger().warning(f'Rejected malformed RIP packet: {error}')
            return

        header = header_from_sonar_message(
            message,
            self.frame_id,
            self.get_clock().now().to_msg(),
        )
        try:
            if isinstance(message, rip.RangeImage):
                self.range_image_publisher.publish(
                    range_image_message(message, header)
                )
                self.pointcloud_publisher.publish(
                    point_cloud_message(message, header)
                )
            elif isinstance(message, rip.BitmapImageGreyscale8):
                self.intensity_image_publisher.publish(
                    bitmap_image_message(message, header)
                )
            else:
                self.get_logger().debug(
                    f'Ignoring unsupported Sonar 3D-15 message '
                    f'{type(message).__name__}'
                )
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected malformed {type(message).__name__}: {error}'
            )

    def destroy_node(self):
        """Close the multicast socket before destroying the ROS node."""
        if hasattr(self, 'sock'):
            sock = self.sock
            del self.sock
            try:
                sock.close()
            except KeyboardInterrupt:
                pass
        return super().destroy_node()


def main(args=None):
    """Run the Sonar 3D-15 live driver."""
    rclpy.init(args=args)
    node = None
    try:
        node = Sonar3DNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
