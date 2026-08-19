"""Conversions from Sonar 3D-15 image messages to ROS-oriented arrays."""

import math

import numpy as np


def _image_shape(message):
    """Return a validated image shape as ``(height, width)``."""
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError(f'invalid image dimensions {width}x{height}')
    return height, width


def range_image_to_meters(message):
    """Convert a vendor range image to a two-dimensional float32 meter array."""
    height, width = _image_shape(message)
    pixels = np.asarray(message.image_pixel_data, dtype=np.float32)
    expected = height * width
    if pixels.size != expected:
        raise ValueError(
            f'range image contains {pixels.size} pixels; expected {expected}'
        )
    return (pixels.reshape(height, width) * float(message.image_pixel_scale)).astype(
        np.float32,
        copy=False,
    )


def _pixel_angles(field_of_view, count):
    """Return centered native image angles for one image dimension."""
    if count == 1:
        return np.zeros(1, dtype=np.float32)
    return np.linspace(
        -field_of_view / 2.0,
        field_of_view / 2.0,
        count,
        dtype=np.float32,
    )


def range_image_to_points(message):
    """
    Convert valid range pixels to ``x,y,z,range,azimuth,elevation`` rows.

    Water Linked uses x-forward, y-right, z-down coordinates. The output flips
    y and z to produce a ROS REP-103 body frame: x-forward, y-left, z-up.
    Angles in the output are expressed in that ROS frame.
    """
    ranges = range_image_to_meters(message)
    height, width = ranges.shape

    horizontal_fov = math.radians(float(message.fov_horizontal))
    vertical_fov = math.radians(float(message.fov_vertical))
    native_yaw = _pixel_angles(horizontal_fov, width)
    native_pitch = _pixel_angles(vertical_fov, height)
    yaw_grid, pitch_grid = np.meshgrid(native_yaw, native_pitch)
    valid = ranges > 0.0
    distance = ranges[valid]
    yaw = yaw_grid[valid]
    pitch = pitch_grid[valid]

    cos_pitch = np.cos(pitch)
    x = distance * cos_pitch * np.cos(yaw)
    y = -distance * cos_pitch * np.sin(yaw)
    z = distance * np.sin(pitch)

    return np.column_stack((x, y, z, distance, -yaw, pitch)).astype(
        np.float32,
        copy=False,
    )


def bitmap_image_to_mono8(message):
    """Convert an 8-bit vendor bitmap into an unmodified mono image array."""
    height, width = _image_shape(message)
    pixels = np.frombuffer(bytes(message.image_pixel_data), dtype=np.uint8)
    expected = height * width
    if pixels.size != expected:
        raise ValueError(
            f'bitmap image contains {pixels.size} pixels; expected {expected}'
        )
    return pixels.reshape(height, width)
