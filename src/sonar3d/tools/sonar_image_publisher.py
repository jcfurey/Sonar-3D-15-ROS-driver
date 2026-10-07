#!/usr/bin/env python3
# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Publish the shared quantitative figure as a display-only RGB image for RViz."""

import argparse

import matplotlib
from matplotlib import pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from sonar3d.msg import ImageMetadata

from sonar_image_science import ImageMatcher, PlotConfig, ScientificFigure, Shots

matplotlib.use('Agg')


class ImagePublisher(Node):
    """Match raw measurements and publish labelled figures at a bounded display rate."""

    def __init__(self, args):
        """Create a headless renderer and relative input/output topics."""
        super().__init__('quantitative_image_publisher', namespace=args.namespace)
        signal_limits = args.signal_limits or ((0, 255) if args.signal_scale == 'encoded'
                                               else (0, 10644))
        config = PlotConfig(tuple(args.range_limits), tuple(signal_limits),
                            args.signal_scale, args.colormap, args.dpi)
        self.plot = ScientificFigure(config)
        self.plot.figure.set_dpi(config.dpi)
        self.matcher, self.shots = ImageMatcher(), Shots()
        self.signature = None
        inputs = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        outputs = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Image, 'quantitative_image', outputs)
        self.image_subscriptions = [self.create_subscription(
            Image, product, lambda msg, product=product: self.image(product, msg), inputs)
            for product in ('range_image', 'intensity_image')]
        self.image_subscriptions.append(self.create_subscription(
            ImageMetadata, 'image_metadata', self.metadata, inputs))
        self.timer = self.create_timer(1 / args.refresh_hz, self.render)

    def accept(self, plane):
        """Retain complete products by original source shot, never by receive time."""
        if plane is not None:
            self.shots.add(plane)

    def image(self, product, message):
        """Reject raw images whose exact metadata or geometry does not agree."""
        try:
            self.accept(self.matcher.add_image(product, message))
        except ValueError as error:
            self.get_logger().warning(f'Image rejected: {error}')

    def metadata(self, message):
        """Accept only metadata for the range and signal inputs this renderer uses."""
        if message.image_type not in (ImageMetadata.RANGE, ImageMetadata.SIGNAL):
            return
        try:
            self.accept(self.matcher.add_metadata(message))
        except ValueError as error:
            self.get_logger().warning(f'Image metadata rejected: {error}')

    def render(self):
        """Sample the latest ready shot without averaging or modifying measurements."""
        frame = self.shots.latest()
        if frame is None or not self.publisher.get_subscription_count():
            return
        signature = (frame['index'], tuple(frame['planes']))
        if signature == self.signature:
            return
        try:
            self.plot.update(frame['planes'])
            rgb = self.plot.rgb8()
        except ValueError as error:
            self.get_logger().warning(f'Figure rejected: {error}')
            return
        header = frame['planes']['range_image'].metadata['header']
        message = Image()
        message.header.frame_id = header['frame_id']
        message.header.stamp.sec = header['stamp']['sec']
        message.header.stamp.nanosec = header['stamp']['nanosec']
        message.height, message.width = rgb.shape[:2]
        message.encoding = 'rgb8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = rgb.tobytes()
        self.publisher.publish(message)
        self.signature = signature

    def destroy_node(self):
        """Release the headless figure with the ROS node."""
        plt.close(self.plot.figure)
        return super().destroy_node()


def main():
    """Run a display-only ROS image publisher with explicit fixed plotting scales."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='sonar3d')
    parser.add_argument('--range-limits', nargs=2, type=float, default=(0, 15))
    parser.add_argument('--signal-limits', nargs=2, type=float)
    parser.add_argument('--signal-scale', choices=('encoded', 'linear'), default='encoded')
    parser.add_argument('--colormap', choices=('cividis', 'viridis', 'magma'), default='cividis')
    parser.add_argument('--dpi', type=int, default=100)
    parser.add_argument('--refresh-hz', type=float, default=3,
                        help='Maximum display rate; raw measurement topics are unaffected')
    args = parser.parse_args(remove_ros_args()[1:])
    if not 0 < args.refresh_hz <= 60:
        parser.error('--refresh-hz must be in (0, 60]')
    rclpy.init()
    node = None
    try:
        node = ImagePublisher(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        parser.exit(1, f'{error}\n')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
