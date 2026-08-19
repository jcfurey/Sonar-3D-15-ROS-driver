"""Unit tests for Sonar 3D-15 image conversions."""

import math

import numpy as np
import pytest

from sonar3d.api.sonar_3d_15_protocol_pb2 import (
    BitmapImageGreyscale8,
    RangeImage,
)
from sonar3d.conversions import (
    bitmap_image_to_mono8,
    range_image_to_meters,
    range_image_to_points,
)


def _range_image():
    """Return a small range image with one invalid pixel."""
    return RangeImage(
        width=3,
        height=2,
        fov_horizontal=90.0,
        fov_vertical=40.0,
        image_pixel_scale=0.01,
        image_pixel_data=[100, 0, 200, 300, 400, 500],
    )


def test_range_image_is_scaled_to_float32_meters():
    ranges = range_image_to_meters(_range_image())

    assert ranges.dtype == np.float32
    assert ranges.shape == (2, 3)
    np.testing.assert_allclose(ranges, [[1.0, 0.0, 2.0], [3.0, 4.0, 5.0]])


def test_point_cloud_retains_range_and_ros_angles():
    points = range_image_to_points(_range_image())

    assert points.dtype == np.float32
    assert points.shape == (5, 6)
    first = points[0]
    np.testing.assert_allclose(first[3], 1.0)
    np.testing.assert_allclose(first[4], math.radians(45.0), atol=1e-6)
    np.testing.assert_allclose(first[5], math.radians(-20.0), atol=1e-6)
    assert first[0] > 0.0
    assert first[1] > 0.0
    assert first[2] < 0.0


def test_bitmap_stays_native_mono8():
    message = BitmapImageGreyscale8(
        width=3,
        height=2,
        image_pixel_data=bytes([0, 1, 2, 253, 254, 255]),
    )

    image = bitmap_image_to_mono8(message)

    assert image.dtype == np.uint8
    assert image.tolist() == [[0, 1, 2], [253, 254, 255]]


@pytest.mark.parametrize('converter', [range_image_to_meters, range_image_to_points])
def test_range_conversion_rejects_incomplete_images(converter):
    message = RangeImage(
        width=2,
        height=2,
        image_pixel_scale=1.0,
        image_pixel_data=[1, 2, 3],
    )

    with pytest.raises(ValueError, match='expected 4'):
        converter(message)


def test_bitmap_conversion_rejects_incomplete_images():
    message = BitmapImageGreyscale8(
        width=2,
        height=2,
        image_pixel_data=b'abc',
    )

    with pytest.raises(ValueError, match='expected 4'):
        bitmap_image_to_mono8(message)
