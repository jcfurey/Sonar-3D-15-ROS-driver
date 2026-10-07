# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Check scientific geometry, units, invalid returns, matching and provenance."""

import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from sonar_image_science import (
    angular_axes, decode_plane, ImageMatcher, linear_strength, metadata_dict,
    PlotConfig, raw_arrays, save_bundle, ScientificFigure, Shots, statistics,
)
from sonar_image_viewer import load_bundle


def header(sec=99, nanos=123):
    """Provide an exact ROS image header without requiring a running graph."""
    return SimpleNamespace(frame_id='sonar', stamp=SimpleNamespace(sec=sec, nanosec=nanos))


def metadata(kind=0):
    """Provide acquisition geometry whose pixel centres can be checked by hand."""
    return SimpleNamespace(
        header=header(), image_type=kind, sequence_id=7, width=3, height=2,
        horizontal_fov_degrees=90.0, vertical_fov_degrees=40.0,
        configured_range_m=15.0, speed_of_sound_mps=1500.0, frequency=5,
        range_pixel_scale_m=0.001 if kind == 0 else 0.0,
        sensor_timestamp_valid=True, sensor_stamp_seconds=42, sensor_stamp_nanoseconds=456,
        driver_version='0.6.0', driver_source_sha256='fixture')


def image(kind=0, bigendian=False, padded=False):
    """Supply a known image, optionally padded and with non-native byte order."""
    dtype = ('>f4' if bigendian else '<f4') if kind == 0 else 'u1'
    data = np.asarray([[3, 0, 5], [1, 2, 4]], dtype=dtype)
    padding = b'xxxx' if padded else b''
    return SimpleNamespace(header=header(), width=3, height=2,
                           encoding='32FC1' if kind == 0 else 'mono8',
                           is_bigendian=bigendian, step=3 * data.dtype.itemsize + len(padding),
                           data=b''.join(row.tobytes() + padding for row in data))


@pytest.mark.parametrize('bigendian,padded', [(False, False), (True, True), (False, True)])
def test_padding_and_byte_order_preserve_measurements(bigendian, padded):
    """Scientific arrays must contain the actual metres, not row-padding bytes."""
    plane = decode_plane('range_image', image(0, bigendian, padded), metadata_dict(metadata()))
    np.testing.assert_array_equal(plane.data, [[3, 0, 5], [1, 2, 4]])
    assert plane.data.dtype.str == '<f4'


def test_angular_centres_and_half_bin_edges():
    """Corner pixel centres agree with REP-103 cloud azimuth and elevation."""
    azimuth, elevation, extent = angular_axes(metadata_dict(metadata()))
    np.testing.assert_array_equal(azimuth, [45, 0, -45])
    np.testing.assert_array_equal(elevation, [20, -20])
    assert extent == (67.5, -67.5, -40.0, 40.0)


def test_single_pixel_axes_have_centre_zero():
    """A single-cell axis has no division by zero or invented off-axis direction."""
    meta = metadata_dict(metadata())
    meta.update(width=1, height=1)
    azimuth, elevation, extent = angular_axes(meta)
    assert azimuth[0] == elevation[0] == 0
    assert extent == (45.0, -45.0, -20.0, 20.0)


@pytest.mark.parametrize('first', ['image', 'metadata'])
def test_matching_requires_exact_headers_in_either_arrival_order(first):
    """A one-nanosecond mismatch must not attach another shot's settings."""
    matcher = ImageMatcher()
    img, meta = image(), metadata()
    meta.header = header(nanos=124)
    assert matcher.add_image('range_image', img) is None
    assert matcher.add_metadata(meta) is None
    matcher = ImageMatcher()
    meta = metadata()
    if first == 'image':
        assert matcher.add_image('range_image', img) is None
        plane = matcher.add_metadata(meta)
    else:
        assert matcher.add_metadata(meta) is None
        plane = matcher.add_image('range_image', img)
    assert plane.metadata['sequence_id'] == 7
    assert plane.metadata['sensor_stamp_seconds'] == 42
    assert plane.metadata['header']['stamp']['sec'] == 99


def test_lost_metadata_cannot_grow_memory_forever():
    """Long recordings or a disconnected producer leave only bounded pending data."""
    matcher = ImageMatcher(capacity=3)
    for sec in range(10):
        img = image()
        img.header = header(sec=sec)
        matcher.add_image('range_image', img)
    assert len(matcher.images) == 3


def test_linear_strength_and_no_return_sentinel():
    """Vendor log coding is relative strength; zero is never decoded as 30."""
    np.testing.assert_array_equal(linear_strength(np.asarray([0, 1, 100, 200, 255], dtype='u1')),
                                  [0, 31, 300, 3000, 10644])


def test_statistics_exclude_invalids_and_report_fixed_scale_clipping():
    """Coverage and range summaries cannot silently include zero/no-return pixels."""
    values = np.asarray([0, np.nan, 2, 5, 20])
    stats = statistics(values, np.isfinite(values) & (values > 0), (0, 15))
    assert stats['valid_pixels'] == 3
    assert stats['median'] == 5
    assert stats['above_colour_scale'] == 1
    assert stats['valid_fraction'] == 0.6
    assert PlotConfig().range_limits == (0, 15)


def test_different_shots_cannot_be_combined():
    """Image labels and intensity must never come from a neighbouring shot."""
    plane = decode_plane('range_image', image(), metadata_dict(metadata()))
    signal = decode_plane('intensity_image', image(1), metadata_dict(metadata(1)))
    signal.metadata['sequence_id'] += 1
    with pytest.raises(ValueError, match='different shots'):
        raw_arrays({'range_image': plane, 'intensity_image': signal})


def test_range_only_stream_remains_viewable_when_signal_is_absent(monkeypatch):
    """Continuous 6 Hz range frames must not restart the missing-signal timeout."""
    clock = [0.0]
    monkeypatch.setattr('sonar_image_science.time.monotonic', lambda: clock[0])
    shots = Shots()
    for index in range(10):
        clock[0] = index / 6
        meta = metadata_dict(metadata())
        meta.update(sequence_id=index, sensor_stamp_nanoseconds=456 + index)
        shots.add(decode_plane('range_image', image(), meta))
        if index >= 2:
            ready = shots.latest()
            assert ready is not None
            assert ready['index'] < index
            assert clock[0] - ready['arrived'] >= 0.25
    signal_meta = metadata_dict(metadata(1))
    signal_meta.update(sequence_id=9, sensor_stamp_nanoseconds=465)
    shots.add(decode_plane('intensity_image', image(1), signal_meta))
    assert shots.latest()['index'] == 9


def test_rviz_raster_preserves_fixed_colours_and_no_return_mask():
    """The RGB view must show fixed metre colours and distinct no-return pixels."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    plane = decode_plane('range_image', image(), metadata_dict(metadata()))
    original = plane.data.copy()
    plot = ScientificFigure(PlotConfig(range_limits=(0, 3)))
    try:
        plot.update({'range_image': plane})
        raster = plot.rgb8()
        assert raster.dtype == np.uint8 and raster.flags.c_contiguous
        azimuth, elevation, _ = angular_axes(plane.metadata)

        def pixel(row, column):
            x, y = plot.axes[0].transData.transform((azimuth[column], elevation[row]))
            return raster[raster.shape[0] - 1 - round(y), round(x)]

        np.testing.assert_array_equal(pixel(0, 1), [217, 223, 229])  # No return.
        np.testing.assert_array_equal(pixel(0, 2), [177, 18, 54])  # Above fixed scale.
        expected = (np.asarray(plt.get_cmap('cividis')(1 / 3)[:3]) * 255).astype('u1')
        np.testing.assert_array_equal(pixel(1, 0), expected)
        saved = raster.copy()
        changed = decode_plane('range_image', image(), metadata_dict(metadata()))
        changed.data[0, 2] = 30
        plot.update({'range_image': changed})
        plot.rgb8()
        np.testing.assert_array_equal(raster, saved)  # No renderer-owned buffer aliasing.
        np.testing.assert_array_equal(plane.data, original)
    finally:
        plt.close(plot.figure)


def test_bundle_preserves_data_settings_checksums_and_can_be_rerendered(tmp_path):
    """PNG is a view; the bundle retains numerical values and acquisition provenance."""
    import matplotlib
    matplotlib.use('Agg')
    range_plane = decode_plane('range_image', image(), metadata_dict(metadata()))
    signal = decode_plane('intensity_image', image(1), metadata_dict(metadata(1)))
    planes = {'range_image': range_plane, 'intensity_image': signal}
    config = PlotConfig(range_limits=(0, 8), signal_limits=(0, 128))
    first, second = tmp_path / 'first', tmp_path / 'second'
    save_bundle(first, planes, config, {'kind': 'fixture', 'sha256': 'abc'})
    loaded, manifest = load_bundle(first)
    np.testing.assert_array_equal(loaded['range_image'].data, range_plane.data)
    assert manifest['rendering']['range_limits'] == [0, 8]
    assert manifest['products']['range_image']['sensor_stamp_nanoseconds'] == 456
    assert manifest['source']['sha256'] == 'abc'
    save_bundle(second, loaded, config, manifest['source'])
    assert hashlib.sha256((first / 'figure.png').read_bytes()).digest() == hashlib.sha256(
        (second / 'figure.png').read_bytes()).digest()
    with pytest.raises(FileExistsError):
        save_bundle(first, planes, config, {})
    broken = json.loads((first / 'metadata.json').read_text())
    broken['arrays']['range_m']['sha256'] = 'bad'
    (first / 'metadata.json').write_text(json.dumps(broken))
    with pytest.raises(ValueError, match='checksum'):
        load_bundle(first)
