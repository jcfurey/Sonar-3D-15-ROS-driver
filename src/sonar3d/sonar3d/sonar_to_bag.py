"""Replay RIP1/RIP2 Sonar 3D-15 recordings as the driver's ROS topics."""

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from sonar3d.ros_messages import (
    bitmap_image_message,
    header_from_sonar_message,
    point_cloud_message,
    range_image_message,
)
import wlsonar.range_image_protocol as rip


class RecordingFormatError(ValueError):
    """Indicate that playback cannot recover the recording's framing."""


def read_recording_messages(path):
    """Yield recognized messages from a length-framed RIP1/RIP2 recording."""
    known_types = (rip.RangeImage, rip.BitmapImageGreyscale8)
    with Path(path).open('rb') as stream:
        while True:
            offset = stream.tell()
            try:
                yield rip.unpack(stream, known_message_types=known_types)
            except EOFError:
                return
            except rip.UnknownProtobufTypeError:
                continue
            except rip.CRCMismatchError:
                # The parser consumed the declared packet length, so framing is intact.
                continue
            except (rip.BadIDError, ValueError) as error:
                raise RecordingFormatError(
                    f'could not parse RIP stream at byte {offset}: {error}'
                ) from error


def decode_packet(packet):
    """Decode one complete RIP1 or RIP2 packet for diagnostics and tests."""
    message = rip.unpackb(
        packet,
        known_message_types=(rip.RangeImage, rip.BitmapImageGreyscale8),
    )
    return type(message).__name__, message


def sensor_time_seconds(message):
    """Return the protobuf sensor timestamp as fractional Unix seconds."""
    message_header = getattr(message, 'header', None)
    stamp = getattr(message_header, 'timestamp', None)
    if stamp is None or not (stamp.seconds or stamp.nanos):
        return None
    return float(stamp.seconds) + float(stamp.nanos) / 1e9


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

    def publish_message(self, message):
        """Publish one decoded message and return whether it was supported."""
        header = header_from_sonar_message(
            message,
            self.frame_id,
            self.get_clock().now().to_msg(),
        )
        if isinstance(message, rip.RangeImage):
            self.range_image_publisher.publish(range_image_message(message, header))
            self.pointcloud_publisher.publish(point_cloud_message(message, header))
        elif isinstance(message, rip.BitmapImageGreyscale8):
            self.intensity_image_publisher.publish(
                bitmap_image_message(message, header)
            )
        else:
            return False
        rclpy.spin_once(self, timeout_sec=0.0)
        return True

    def publish_recording(self, messages, realtime_factor):
        """Publish messages while reproducing nonnegative sensor-time gaps."""
        previous_stamp = None
        published = 0
        for message in messages:
            stamp = sensor_time_seconds(message)
            if stamp is not None and previous_stamp is not None:
                delay = (stamp - previous_stamp) / realtime_factor
                if delay > 0.0:
                    time.sleep(delay)
            try:
                supported = self.publish_message(message)
            except ValueError as error:
                self.get_logger().warning(
                    f'Rejected malformed {type(message).__name__}: {error}'
                )
                continue
            if not supported:
                continue
            if stamp is not None:
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
    parsed, _ = _argument_parser().parse_known_args(args)
    if parsed.realtime_factor <= 0.0:
        raise SystemExit('--realtime-factor must be greater than zero')

    rclpy.init(args=None)
    node = None
    try:
        node = SonarPlaybackNode(parsed.frame_id)
        messages = read_recording_messages(parsed.file)
        count = node.publish_recording(messages, parsed.realtime_factor)
        node.get_logger().info(f'Published {count} Sonar 3D-15 messages')
    except (FileNotFoundError, RecordingFormatError) as error:
        raise SystemExit(str(error)) from error
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
