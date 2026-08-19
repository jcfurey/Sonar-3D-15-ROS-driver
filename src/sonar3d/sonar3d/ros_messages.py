"""Build ROS messages from decoded Sonar 3D-15 protobuf messages."""

import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField

from sonar3d.conversions import (
    bitmap_image_to_mono8,
    range_image_to_meters,
    range_image_to_points,
)
from std_msgs.msg import Header


POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='range', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
]


def header_from_sonar_message(message, frame_id, fallback_stamp):
    """Build a header using the sensor timestamp when it is available."""
    header = Header()
    header.frame_id = frame_id
    message_header = getattr(message, 'header', None)
    stamp = getattr(message_header, 'timestamp', None)
    if stamp is not None and (stamp.seconds or stamp.nanos):
        header.stamp.sec = int(stamp.seconds)
        header.stamp.nanosec = int(stamp.nanos)
    else:
        header.stamp = fallback_stamp
    return header


def range_image_message(message, header):
    """Build a ``32FC1`` image whose pixels contain range in meters."""
    ranges = range_image_to_meters(message)
    image = Image()
    image.header = header
    image.height, image.width = ranges.shape
    image.encoding = '32FC1'
    image.is_bigendian = False
    image.step = image.width * np.dtype(np.float32).itemsize
    image.data = ranges.tobytes()
    return image


def bitmap_image_message(message, header):
    """Build a native 8-bit mono image without scaling or normalization."""
    pixels = bitmap_image_to_mono8(message)
    image = Image()
    image.header = header
    image.height, image.width = pixels.shape
    image.encoding = 'mono8'
    image.is_bigendian = False
    image.step = image.width
    image.data = pixels.tobytes()
    return image


def point_cloud_message(message, header):
    """Build a dense cloud retaining range and both per-return angles."""
    points = range_image_to_points(message)
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = POINT_FIELDS
    cloud.is_bigendian = False
    cloud.point_step = 6 * np.dtype(np.float32).itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud
