"""Unit tests for Sonar 3D-15 RIP1/RIP2 recording decoding."""

import pytest

from sonar3d.sonar_to_bag import (
    decode_packet,
    read_recording_messages,
    RecordingFormatError,
)
import wlsonar.range_image_protocol as rip


def _range_packet(sequence_id, protocol):
    """Build one valid framed range-image packet."""
    image = rip.RangeImage(
        width=2,
        height=1,
        fov_horizontal=20.0,
        fov_vertical=10.0,
        image_pixel_scale=0.01,
        image_pixel_data=[100, 200],
    )
    image.header.sequence_id = sequence_id
    image.header.timestamp.seconds = 1000 + sequence_id
    return rip.packb(image, protocol=protocol)


@pytest.mark.parametrize(
    'protocol',
    [rip.ProtocolVersion.RIP1, rip.ProtocolVersion.RIP2],
)
def test_decode_packet_supports_both_protocol_versions(protocol):
    packet = _range_packet(7, protocol)

    message_type, message = decode_packet(packet)

    assert message_type == 'RangeImage'
    assert message.header.sequence_id == 7


def test_recording_reader_supports_mixed_rip1_and_rip2(tmp_path):
    recording = tmp_path / 'survey.sonar'
    recording.write_bytes(
        _range_packet(1, rip.ProtocolVersion.RIP1)
        + _range_packet(2, rip.ProtocolVersion.RIP2)
    )

    messages = list(read_recording_messages(recording))

    assert [message.header.sequence_id for message in messages] == [1, 2]


def test_recording_reader_skips_crc_failure_with_intact_framing(tmp_path):
    damaged = bytearray(_range_packet(1, rip.ProtocolVersion.RIP1))
    damaged[8] ^= 0x01
    recording = tmp_path / 'survey.sonar'
    recording.write_bytes(
        damaged + _range_packet(2, rip.ProtocolVersion.RIP2)
    )

    messages = list(read_recording_messages(recording))

    assert [message.header.sequence_id for message in messages] == [2]


def test_recording_reader_reports_lost_framing(tmp_path):
    recording = tmp_path / 'survey.sonar'
    recording.write_bytes(b'not-a-rip-packet')

    with pytest.raises(RecordingFormatError, match='byte 0'):
        list(read_recording_messages(recording))
