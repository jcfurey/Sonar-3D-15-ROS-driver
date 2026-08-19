"""Replay a Sonar 3D-15 recording as the driver's ROS topic contract."""

import argparse
from pathlib import Path
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from sonar3d.api.inspect_sonar_data import decode_protobuf_packet, parse_rip1_packet
from sonar3d.ros_messages import (
    bitmap_image_message,
    header_from_sonar_message,
    point_cloud_message,
    range_image_message,
)


def read_rip1_packets(path):
    """Return complete RIP1 packets from a Sonar 3D-15 recording."""
    content = Path(path).read_bytes()
    packets = []
    cursor = 0
    while cursor < len(content):
        start = content.find(b'RIP1', cursor)
        if start < 0 or start + 8 > len(content):
            break
        packet_size = struct.unpack_from('<I', content, start + 4)[0]
        if packet_size < 12 or start + packet_size > len(content):
            cursor = start + 4
            continue
        packets.append(content[start:start + packet_size])
        cursor = start + packet_size
    return packets


def sensor_time_seconds(message):
    """Return the protobuf sensor timestamp as fractional Unix seconds."""
    stamp = message.header.timestamp
    return float(stamp.seconds) + float(stamp.nanos) / 1e9


def decode_packet(packet):
    """Decode one complete RIP1 packet into its typed protobuf message."""
    payload = parse_rip1_packet(packet)
    if payload is None:
        return None
    return decode_protobuf_packet(payload)


class SonarPlaybackNode(Node):
    """Publish decoded recording packets with their original timing."""

    def __init__(self, frame_id):
        """Create playback publishers matching the live driver."""
        super().__init__('sonar3d_playback')
        self.frame_id = frame_id
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
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            'sonar_point_cloud',
            qos_profile_sensor_data,
        )

    def publish_message(self, message_type, message):
        """Publish one decoded message and return whether it was supported."""
        header = header_from_sonar_message(
            message,
            self.frame_id,
            self.get_clock().now().to_msg(),
        )
        if message_type == 'RangeImage':
            self.range_image_publisher.publish(range_image_message(message, header))
            self.pointcloud_publisher.publish(point_cloud_message(message, header))
        elif message_type == 'BitmapImageGreyscale8':
            self.intensity_image_publisher.publish(bitmap_image_message(message, header))
        else:
            return False
        rclpy.spin_once(self, timeout_sec=0.0)
        return True

    def publish_recording(self, packets, realtime_factor):
        """Publish all packets while reproducing nonnegative sensor-time gaps."""
        previous_stamp = None
        published = 0
        for packet in packets:
            result = decode_packet(packet)
            if not result:
                continue
            message_type, message = result
            if message_type not in ('RangeImage', 'BitmapImageGreyscale8'):
                continue
            stamp = sensor_time_seconds(message)
            if previous_stamp is not None:
                delay = (stamp - previous_stamp) / realtime_factor
                if delay > 0.0:
                    time.sleep(delay)
            if not self.publish_message(message_type, message):
                continue
            previous_stamp = stamp
            published += 1
        return published


def _argument_parser():
    """Build the command-line parser for recording playback."""
    parser = argparse.ArgumentParser(
        description='Publish a Sonar 3D-15 .sonar recording as ROS 2 messages.'
    )
    parser.add_argument('--file', required=True, help='Sonar .sonar recording')
    parser.add_argument(
        '--realtime-factor',
        type=float,
        default=1.0,
        help='Playback multiplier; 2.0 is twice real time',
    )
    parser.add_argument('--frame-id', default='sonar3d_link')
    return parser


def main(args=None):
    """Replay a recording until all recognized messages have been published."""
    parsed = _argument_parser().parse_args(args)
    if parsed.realtime_factor <= 0.0:
        raise SystemExit('--realtime-factor must be greater than zero')
    packets = read_rip1_packets(parsed.file)
    if not packets:
        raise SystemExit(f'no complete RIP1 packets found in {parsed.file}')

    rclpy.init(args=None)
    node = None
    try:
        node = SonarPlaybackNode(parsed.frame_id)
        count = node.publish_recording(packets, parsed.realtime_factor)
        node.get_logger().info(f'Published {count} Sonar 3D-15 messages')
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
