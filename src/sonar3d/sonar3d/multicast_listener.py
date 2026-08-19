"""ROS 2 live driver for Water Linked Sonar 3D-15 multicast data."""

import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from sonar3d.api.inspect_sonar_data import decode_protobuf_packet, parse_rip1_packet
from sonar3d.http_api import configure_sonar
from sonar3d.ros_messages import (
    bitmap_image_message,
    header_from_sonar_message,
    point_cloud_message,
    range_image_message,
)


class Sonar3DNode(Node):
    """Receive Sonar 3D-15 datagrams and publish native ROS products."""

    DEFAULT_MULTICAST_GROUP = '224.0.0.96'
    DEFAULT_MULTICAST_PORT = 4747
    MAX_DATAGRAM_SIZE = 65535

    def __init__(self):
        """Configure the sonar, multicast socket, and ROS publishers."""
        super().__init__('sonar3d_driver')

        self.declare_parameter('sonar_ip', '192.168.194.96')
        self.declare_parameter('frame_id', 'sonar3d_link')
        self.declare_parameter('speed_of_sound', 0)
        self.declare_parameter('configure_sonar', True)
        self.declare_parameter('http_timeout', 5.0)
        self.declare_parameter('multicast_group', self.DEFAULT_MULTICAST_GROUP)
        self.declare_parameter('multicast_port', self.DEFAULT_MULTICAST_PORT)
        self.declare_parameter('multicast_interface', '0.0.0.0')
        self.declare_parameter('poll_period', 0.01)
        self.declare_parameter('max_packets_per_spin', 32)

        self.sonar_ip = self.get_parameter('sonar_ip').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_packets_per_spin = self.get_parameter('max_packets_per_spin').value

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

        multicast_group = self.get_parameter('multicast_group').value
        multicast_port = self.get_parameter('multicast_port').value
        multicast_interface = self.get_parameter('multicast_interface').value
        self.sock = self._open_multicast_socket(
            multicast_group,
            multicast_port,
            multicast_interface,
        )
        poll_period = self.get_parameter('poll_period').value
        self.timer = self.create_timer(poll_period, self._poll_socket)
        self.get_logger().info(
            f'Listening for Sonar 3D-15 RIP1 packets on '
            f'{multicast_group}:{multicast_port}'
        )

    @staticmethod
    def _open_multicast_socket(group, port, interface):
        """Create a nonblocking UDP multicast receiver."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', port))
        membership = socket.inet_aton(group) + socket.inet_aton(interface)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sock.setblocking(False)
        return sock

    def _configure_sonar(self):
        """Apply requested HTTP settings before listening for datagrams."""
        speed_of_sound = self.get_parameter('speed_of_sound').value
        http_timeout = self.get_parameter('http_timeout').value
        try:
            applied = configure_sonar(
                self.sonar_ip,
                speed_of_sound=speed_of_sound,
                timeout=http_timeout,
            )
            self.get_logger().info(f'Configured sonar settings: {", ".join(applied)}')
        except Exception as error:  # requests exposes several transport exceptions
            self.get_logger().warning(
                f'Could not configure Sonar 3D-15 at {self.sonar_ip}: {error}'
            )

    def _poll_socket(self):
        """Drain a bounded batch without blocking the ROS executor."""
        for _ in range(self.max_packets_per_spin):
            try:
                data, address = self.sock.recvfrom(self.MAX_DATAGRAM_SIZE)
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
            self.get_logger().debug(
                f'Ignoring Sonar 3D-15 packet from unexpected source {source_ip}'
            )
            return

        payload = parse_rip1_packet(data)
        if payload is None:
            self.get_logger().warning('Rejected malformed Sonar 3D-15 RIP1 packet')
            return
        result = decode_protobuf_packet(payload)
        if not result:
            self.get_logger().warning('Could not decode Sonar 3D-15 protobuf packet')
            return

        message_type, message = result
        header = header_from_sonar_message(
            message,
            self.frame_id,
            self.get_clock().now().to_msg(),
        )
        try:
            if message_type == 'RangeImage':
                self.range_image_publisher.publish(range_image_message(message, header))
                self.pointcloud_publisher.publish(point_cloud_message(message, header))
            elif message_type == 'BitmapImageGreyscale8':
                self.intensity_image_publisher.publish(
                    bitmap_image_message(message, header)
                )
            else:
                self.get_logger().debug(
                    f'Ignoring unsupported Sonar 3D-15 message {message_type}'
                )
        except ValueError as error:
            self.get_logger().warning(f'Rejected malformed {message_type}: {error}')

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
