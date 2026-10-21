# Copyright 2026 Sonar 3D-15 ROS Driver contributors
# SPDX-License-Identifier: MIT
# Exact RIP datagram, before decoding or sequence filtering. Includes framing,
# CRC and every protobuf field, including unknown message types and fields.
# Live header stamp is host receipt time. Native recording playback/conversion
# uses its acquisition-time reference (the previous/first known stamp for
# unknown or damaged messages); .sonar files contain no host receipt timestamps.
std_msgs/Header header
# Sender IPv4 address; empty for recordings which do not store sender addresses.
string source_address
# Monotonic input order per driver instance or recording. Bag readers may sort
# by acquisition stamp; sort this index to reconstruct the original RIP stream.
uint64 packet_index
uint8[] data
