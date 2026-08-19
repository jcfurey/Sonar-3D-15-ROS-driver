"""Unit tests for the Sonar 3D-15 ROS message contract."""

from builtin_interfaces.msg import Time
import numpy as np

from sonar3d.ros_messages import (
    bitmap_image_message,
    header_from_sonar_message,
    point_cloud_message,
    range_image_message,
)
from wlsonar.range_image_protocol import (
    BitmapImageGreyscale8,
    RangeImage,
)


def _range_image():
    """Return a compact valid range image."""
    message = RangeImage(
        width=2,
        height=2,
        fov_horizontal=20.0,
        fov_vertical=10.0,
        image_pixel_scale=0.5,
        image_pixel_data=[0, 2, 4, 6],
    )
    message.header.timestamp.seconds = 123
    message.header.timestamp.nanos = 456
    return message


def test_sensor_timestamp_and_range_image_contract():
    source = _range_image()
    header = header_from_sonar_message(source, 'sonar_link', Time(sec=9))

    image = range_image_message(source, header)

    assert image.header.frame_id == 'sonar_link'
    assert image.header.stamp.sec == 123
    assert image.header.stamp.nanosec == 456
    assert image.encoding == '32FC1'
    assert image.step == 8
    np.testing.assert_allclose(
        np.frombuffer(image.data, dtype=np.float32),
        [0.0, 1.0, 2.0, 3.0],
    )


def test_point_cloud_retains_range_and_angles():
    source = _range_image()
    header = header_from_sonar_message(source, 'sonar_link', Time())

    cloud = point_cloud_message(source, header)

    assert cloud.width == 3
    assert cloud.point_step == 24
    assert cloud.row_step == 72
    assert [field.name for field in cloud.fields] == [
        'x',
        'y',
        'z',
        'range',
        'azimuth',
        'elevation',
    ]
    assert len(cloud.data) == cloud.row_step


def test_native_bitmap_contract_and_clock_fallback():
    source = BitmapImageGreyscale8(
        width=2,
        height=2,
        image_pixel_data=bytes([0, 64, 128, 255]),
    )
    fallback = Time(sec=7, nanosec=8)
    header = header_from_sonar_message(source, 'sonar_link', fallback)

    image = bitmap_image_message(source, header)

    assert image.header.stamp == fallback
    assert image.encoding == 'mono8'
    assert image.step == 2
    assert bytes(image.data) == bytes([0, 64, 128, 255])
