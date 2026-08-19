"""Unit tests for Sonar 3D-15 recording framing and decoding."""

import struct
import zlib

from sonar3d.api.sonar_3d_15_protocol_pb2 import Packet, RangeImage
from sonar3d.sonar_to_bag import decode_packet, read_rip1_packets


def _range_packet(sequence_id):
    """Build one valid RIP1-framed range-image packet."""
    image = RangeImage(
        width=2,
        height=1,
        fov_horizontal=20.0,
        fov_vertical=10.0,
        image_pixel_scale=0.01,
        image_pixel_data=[100, 200],
    )
    image.header.sequence_id = sequence_id
    image.header.timestamp.seconds = 1000 + sequence_id
    packet = Packet()
    packet.msg.Pack(image)
    payload = packet.SerializeToString()
    packet_size = 8 + len(payload) + 4
    framed = b'RIP1' + struct.pack('<I', packet_size) + payload
    return framed + struct.pack('<I', zlib.crc32(framed) & 0xffffffff)


def test_recording_reader_uses_packet_lengths(tmp_path):
    first = _range_packet(1)
    second = _range_packet(2)
    recording = tmp_path / 'survey.sonar'
    recording.write_bytes(b'file-prefix' + first + second + b'trailer')

    packets = read_rip1_packets(recording)

    assert packets == [first, second]
    message_type, message = decode_packet(packets[1])
    assert message_type == 'RangeImage'
    assert message.header.sequence_id == 2


def test_recording_reader_skips_invalid_candidate(tmp_path):
    valid = _range_packet(3)
    recording = tmp_path / 'survey.sonar'
    recording.write_bytes(b'RIP1' + struct.pack('<I', 999999) + valid)

    assert read_rip1_packets(recording) == [valid]
