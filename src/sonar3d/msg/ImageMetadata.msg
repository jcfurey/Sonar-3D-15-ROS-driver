# Copyright 2026 Sonar 3D-15 ROS Driver contributors
# SPDX-License-Identifier: MIT
#
# Geometry and acquisition settings for one raw ROS image. Match by image_type
# and the exact header stamp/frame_id; header uses the same ROS clock as Image.
# Images are upright: row 0 is highest elevation, column 0 is positive (left)
# REP-103 azimuth. Vendor pixel centres span the reported field of view.
uint8 RANGE=0
uint8 SIGNAL=1
uint8 SHADED=2
std_msgs/Header header
uint8 image_type
uint32 sequence_id
uint32 width
uint32 height
float32 horizontal_fov_degrees
float32 vertical_fov_degrees
float32 configured_range_m
float32 speed_of_sound_mps
# Preserve the vendor field without inferring a unit or acoustic carrier rate.
uint32 frequency
# Metres per integer pixel for RANGE; zero (not applicable) for bitmaps.
float32 range_pixel_scale_m
# Preserve the original sonar clock even when header is re-anchored/receive-time.
# This clock can be unsynchronized; its epoch is not verified acquisition UTC.
bool sensor_timestamp_valid
int64 sensor_stamp_seconds
int32 sensor_stamp_nanoseconds
string driver_version
string driver_source_sha256
