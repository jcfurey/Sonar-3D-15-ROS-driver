# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Exercise compressed/raw display delivery, headers, colours and draw count."""

import io
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image as PillowImage
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage, Image

from sonar_image_publisher import ImagePublisher
from sonar_image_science import decode_plane, metadata_dict
from test_image_science import image, metadata


def test_compressed_display_and_late_raw_subscriber_share_one_draw_per_refresh(monkeypatch):
    """A compressed-only viewer gets a lossless image; raw remains available on demand."""
    # No /clock is published: a paused replay must still serve a new viewer.
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    renderer = ImagePublisher(SimpleNamespace(
        namespace='/sonar3d_render_test', signal_limits=None, signal_scale='encoded',
        range_limits=(0, 15), colormap='cividis', dpi=50, refresh_hz=20))
    observer = rclpy.create_node('display_observer', namespace='/sonar3d_render_test')
    executor = SingleThreadedExecutor()
    executor.add_node(renderer)
    executor.add_node(observer)
    compressed, raw, draws = [], [], []
    draw = renderer.plot.figure.canvas.draw

    def counted_draw(*args, **kwargs):
        draws.append(True)
        return draw(*args, **kwargs)

    monkeypatch.setattr(renderer.plot.figure.canvas, 'draw', counted_draw)

    def wait(predicate):
        end = time.monotonic() + 5
        while time.monotonic() < end and not predicate():
            executor.spin_once(timeout_sec=0.01)
        assert predicate()

    try:
        observer.create_subscription(
            CompressedImage, 'quantitative_image/compressed', compressed.append, 10)
        renderer.accept(decode_plane('range_image', image(), metadata_dict(metadata())))
        wait(lambda: len(compressed) == 1)
        assert renderer.publisher.get_subscription_count() == 0
        assert len(draws) == 1
        assert compressed[0].format == 'rgb8; png compressed bgr8'
        assert compressed[0].header.frame_id == 'sonar'
        assert compressed[0].header.stamp.sec == 99
        assert compressed[0].header.stamp.nanosec == 123
        observer.create_subscription(Image, 'quantitative_image', raw.append, 10)
        # A paused recording must also serve a transport that subscribes later.
        wait(lambda: len(raw) == 1)
        assert len(draws) == 2
        assert raw[0].header == compressed[0].header
        decoded = np.asarray(PillowImage.open(io.BytesIO(bytes(compressed[0].data))))
        expected = np.frombuffer(bytes(raw[0].data), dtype='u1').reshape(
            raw[0].height, raw[0].width, 3)
        np.testing.assert_array_equal(decoded, expected)
        assert len(compressed[0].data) < len(raw[0].data) / 4
    finally:
        executor.shutdown()
        renderer.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
