# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Quantitative image decoding, exact metadata matching, figures and data bundles."""

from collections import OrderedDict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np


PRODUCTS = {0: 'range_image', 1: 'intensity_image', 2: 'shaded_image'}
METADATA_FIELDS = (
    'image_type', 'sequence_id', 'width', 'height', 'horizontal_fov_degrees',
    'vertical_fov_degrees', 'configured_range_m', 'speed_of_sound_mps', 'frequency',
    'range_pixel_scale_m', 'sensor_timestamp_valid', 'sensor_stamp_seconds',
    'sensor_stamp_nanoseconds', 'driver_version', 'driver_source_sha256',
)


@dataclass(frozen=True)
class PlotConfig:
    """Fixed scales shared by live and offline rendering, independent of data."""

    range_limits: tuple = (0.0, 15.0)
    signal_limits: tuple = (0.0, 255.0)
    signal_scale: str = 'encoded'
    colormap: str = 'cividis'
    dpi: int = 160

    def __post_init__(self):
        """Reject ambiguous, nonfinite, or reversed plotting scales."""
        if self.signal_scale not in ('encoded', 'linear'):
            raise ValueError('signal_scale must be encoded or linear')
        for limits in (self.range_limits, self.signal_limits):
            if len(limits) != 2 or not np.all(np.isfinite(limits)) or limits[0] >= limits[1]:
                raise ValueError('colour limits must be two finite, increasing values')
        if self.range_limits[0] < 0 or self.signal_limits[0] < 0 or self.dpi <= 0:
            raise ValueError('limits must be nonnegative and dpi positive')


@dataclass
class Plane:
    """An unchanged upright ROS image and its exactly matched acquisition metadata."""

    product: str
    data: np.ndarray
    metadata: dict

    @property
    def shot_key(self):
        """Identify the shot using source sequence/clock, not replay receive times."""
        meta = self.metadata
        return (meta['header']['frame_id'], meta['sequence_id'],
                meta['sensor_timestamp_valid'], meta['sensor_stamp_seconds'],
                meta['sensor_stamp_nanoseconds'])


class Shots:
    """Group exactly matched products and select the newest ready source shot."""

    def __init__(self):
        """Track range-frame ordinals with bounded storage for incomplete shots."""
        self.frames = OrderedDict()
        self.count = 0

    def add(self, plane):
        """Keep products from one source clock/sequence together, in either order."""
        frame = self.frames.setdefault(plane.shot_key, {
            'planes': {}, 'index': None, 'arrived': time.monotonic()})
        frame['planes'][plane.product] = plane
        if plane.product == 'range_image' and frame['index'] is None:
            frame['index'] = self.count
            self.count += 1
        self.frames.move_to_end(plane.shot_key)
        while len(self.frames) > 128:
            self.frames.popitem(last=False)
        return frame

    def latest(self):
        """Allow range-only viewing after a short matching-signal grace period."""
        now = time.monotonic()
        ready = [frame for frame in self.frames.values() if frame['index'] is not None and
                 ('intensity_image' in frame['planes'] or now - frame['arrived'] >= 0.25)]
        return max(ready, key=lambda item: item['index'], default=None)


def header_dict(header):
    """Preserve ROS timestamps as integers, including all nanoseconds."""
    return {'frame_id': header.frame_id,
            'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec}}


def metadata_dict(message):
    """Copy all acquisition fields without assigning undocumented units."""
    return {'header': header_dict(message.header),
            **{name: getattr(message, name) for name in METADATA_FIELDS}}


def image_key(product, header):
    """Match images to metadata by product, frame and exact ROS timestamp."""
    return (product, header.frame_id, header.stamp.sec, header.stamp.nanosec)


def decode_plane(product, image, metadata):
    """Decode row padding/endianness and reject mismatched metadata or geometry."""
    if product != PRODUCTS.get(metadata['image_type']):
        raise ValueError('image product does not match metadata')
    if header_dict(image.header) != metadata['header']:
        raise ValueError('image header does not exactly match metadata')
    if (image.width, image.height) != (metadata['width'], metadata['height']):
        raise ValueError('image dimensions do not match metadata')
    if image.width <= 0 or image.height <= 0:
        raise ValueError('empty image')
    expected_encoding = '32FC1' if product == 'range_image' else 'mono8'
    if image.encoding != expected_encoding:
        raise ValueError(f'{product} requires {expected_encoding}, got {image.encoding}')
    dtype = np.dtype(('>f4' if image.is_bigendian else '<f4')
                     if product == 'range_image' else 'u1')
    if image.step < image.width * dtype.itemsize or len(image.data) != image.height * image.step:
        raise ValueError('invalid image stride or data size')
    data = np.ndarray((image.height, image.width), dtype=dtype, buffer=bytes(image.data),
                      strides=(image.step, dtype.itemsize))
    data = data.astype('<f4' if product == 'range_image' else 'u1', copy=True)
    angular_axes(metadata)
    return Plane(product, data, metadata)


class ImageMatcher:
    """Bounded exact matching accepts either arrival order and never guesses time."""

    def __init__(self, capacity=128):
        """Bound memory independently of recording duration or network losses."""
        self.capacity = capacity
        self.images = OrderedDict()
        self.metadata = OrderedDict()

    def _match(self, key):
        if key in self.images and key in self.metadata:
            return decode_plane(key[0], self.images.pop(key), self.metadata.pop(key))
        return None

    def _remember(self, cache, key, value):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.capacity:
            cache.popitem(last=False)

    def add_image(self, product, message):
        """Return a plane only when its exact metadata has also arrived."""
        key = image_key(product, message.header)
        self._remember(self.images, key, message)
        return self._match(key)

    def add_metadata(self, message):
        """Ignore unknown products; pair recognized metadata without a tolerance."""
        product = PRODUCTS.get(message.image_type)
        if product is None:
            return None
        key = image_key(product, message.header)
        self._remember(self.metadata, key, metadata_dict(message))
        return self._match(key)


def angular_axes(metadata):
    """Return REP-103 pixel centres and half-bin edges in degrees, upright."""
    width, height = metadata['width'], metadata['height']
    horizontal, vertical = (metadata['horizontal_fov_degrees'],
                            metadata['vertical_fov_degrees'])
    if width <= 0 or height <= 0 or not np.all(np.isfinite([horizontal, vertical])):
        raise ValueError('invalid angular image geometry')
    if not 0 < horizontal <= 360 or not 0 < vertical <= 360:
        raise ValueError('angular fields of view must be in (0, 360] degrees')
    azimuth = np.linspace(horizontal / 2, -horizontal / 2, width) if width > 1 else np.zeros(1)
    elevation = np.linspace(vertical / 2, -vertical / 2, height) if height > 1 else np.zeros(1)
    dx = horizontal / (width - 1) if width > 1 else horizontal
    dy = vertical / (height - 1) if height > 1 else vertical
    extent = (azimuth[0] + dx / 2, azimuth[-1] - dx / 2,
              elevation[-1] - dy / 2, elevation[0] + dy / 2)
    return azimuth, elevation, extent


def linear_strength(codes):
    """Decode vendor-relative linear strength; zero stays the no-return sentinel."""
    table = np.asarray([0] + [round(30 * 10 ** (p / 100)) for p in range(1, 256)],
                       dtype='<f4')
    return table[codes]


def raw_arrays(planes):
    """Preserve numerical ROS measurements and explicit masks/coordinate centres."""
    if 'range_image' not in planes:
        raise ValueError('a matched range image is required')
    reference = planes['range_image']
    azimuth, elevation, _ = angular_axes(reference.metadata)
    arrays = {'range_m': reference.data,
              'valid_range': np.isfinite(reference.data) & (reference.data > 0),
              'azimuth_deg': azimuth, 'elevation_deg': elevation}
    for product, plane in planes.items():
        if plane.shot_key != reference.shot_key:
            raise ValueError('products are from different shots')
        for field in ('width', 'height', 'horizontal_fov_degrees', 'vertical_fov_degrees'):
            if plane.metadata[field] != reference.metadata[field]:
                raise ValueError('products do not share one angular image grid')
        if product == 'intensity_image':
            arrays['signal_code'] = plane.data
            arrays['signal_strength_relative'] = linear_strength(plane.data)
            arrays['valid_signal'] = plane.data != 0
        elif product == 'shaded_image':
            arrays['shaded_code'] = plane.data
    return arrays


def utc_clock(seconds, nanoseconds):
    """Format a clock reading without losing nanoseconds or claiming NTP sync."""
    try:
        prefix = datetime.fromtimestamp(seconds, timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
        return f'{prefix}.{nanoseconds:09d} UTC'
    except (ValueError, OverflowError, OSError):
        return f'{seconds}s + {nanoseconds}ns'


def statistics(values, valid, limits):
    """Exclude no-return data from statistics and report clipping separately."""
    selected = values[valid]
    return {'total_pixels': int(values.size), 'valid_pixels': int(selected.size),
            'valid_fraction': float(selected.size / values.size),
            'below_colour_scale': int(np.count_nonzero(selected < limits[0])),
            'above_colour_scale': int(np.count_nonzero(selected > limits[1])),
            'minimum': float(np.min(selected)) if selected.size else None,
            'maximum': float(np.max(selected)) if selected.size else None,
            'median': float(np.median(selected)) if selected.size else None}


class ScientificFigure:
    """Fixed-scale angular plots shared by live viewing and offline exports."""

    def __init__(self, config):
        """Create labelled, perceptually uniform panels with distinct invalid data."""
        import matplotlib.pyplot as plt
        # Reset plotting rc settings, preserving the active GUI/backend choice.
        plt.style.use('default')
        self.config = config
        self.figure, self.axes = plt.subplots(1, 2, figsize=(13, 5.8))
        self.figure.subplots_adjust(left=0.065, right=0.965, bottom=0.31,
                                    top=0.72, wspace=0.22)
        self.artists = []
        cmap = plt.get_cmap(config.colormap).copy()
        cmap.set_bad('#d9dfe5')
        cmap.set_under('#163352')
        cmap.set_over('#b11236')
        for axis, limits, label in zip(
                self.axes, (config.range_limits, config.signal_limits),
                ('Slant range (m)', 'Signal code (vendor log encoding)' if
                 config.signal_scale == 'encoded' else 'Signal strength (relative units)')):
            artist = axis.imshow(np.ma.masked_all((2, 2)), origin='upper',
                                 interpolation='nearest', cmap=cmap,
                                 vmin=limits[0], vmax=limits[1], aspect='equal')
            axis.set_xlabel('Azimuth (°; positive left)')
            axis.set_ylabel('Elevation (°; positive up)')
            colorbar = self.figure.colorbar(
                artist, ax=axis, orientation='horizontal', fraction=0.08, pad=0.30, extend='both')
            colorbar.set_label(label)
            self.artists.append(artist)
        self.detail = self.figure.text(0.065, 0.84, '', fontsize=9, va='top')
        self.figure.text(0.065, 0.17,
                         'Grey: no valid return    Blue/red: outside fixed colour scale\n'
                         'Strongest return per angular cell; no spatial interpolation. '
                         'Signal is not calibrated backscatter.', fontsize=9)

    def update(self, planes):
        """Render one shot without normalizing by its extrema or mixing shots."""
        arrays = raw_arrays(planes)
        meta = planes['range_image'].metadata
        _, _, extent = angular_axes(meta)
        values = [arrays['range_m'], arrays.get(
            'signal_code' if self.config.signal_scale == 'encoded' else
            'signal_strength_relative', np.zeros_like(arrays['range_m']))]
        masks = [arrays['valid_range'], arrays.get('valid_signal',
                                                   np.zeros_like(arrays['valid_range']))]
        for axis, artist, data, valid, name, limits in zip(
                self.axes, self.artists, values, masks, ('Range', 'Signal strength'),
                (self.config.range_limits, self.config.signal_limits)):
            artist.set_data(np.ma.array(data, mask=~valid))
            artist.set_extent(extent)
            axis.set_xlim(extent[0], extent[1])
            axis.set_ylim(extent[2], extent[3])
            stats = statistics(data, valid, limits)
            clipped = stats['below_colour_scale'] + stats['above_colour_scale']
            missing = name == 'Signal strength' and 'intensity_image' not in planes
            suffix = 'unavailable' if missing else (
                f"{100 * stats['valid_fraction']:.1f}% valid; {clipped} outside scale")
            axis.set_title(f'{name} · {suffix}', fontsize=11)
        self.figure.suptitle(f"Sonar 3D-15 · shot {meta['sequence_id']} · "
                             f"{meta['width']} × {meta['height']}", fontsize=15, y=0.96)
        sensor_clock = utc_clock(meta['sensor_stamp_seconds'], meta['sensor_stamp_nanoseconds'])
        sensor_clock = sensor_clock if meta['sensor_timestamp_valid'] else 'unavailable'
        ros_stamp = meta['header']['stamp']
        self.detail.set_text(
            f'Sonar clock: {sensor_clock} (synchronization unverified)\n'
            f"ROS image clock: {utc_clock(ros_stamp['sec'], ros_stamp['nanosec'])}\n"
            f"FOV centres: {meta['horizontal_fov_degrees']:g}° × "
            f"{meta['vertical_fov_degrees']:g}°; configured range: "
            f"{meta['configured_range_m']:g} m; sound speed: {meta['speed_of_sound_mps']:.3f} m/s")
        self.figure.canvas.draw_idle()

    def rgb8(self):
        """Return an owned RGB raster of the labelled figure for an RViz Image display."""
        self.figure.canvas.draw()
        return np.asarray(self.figure.canvas.buffer_rgba())[:, :, :3].copy()


def sha256_file(path):
    """Hash inputs incrementally so provenance does not depend on filenames."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save_bundle(directory, planes, config, source):
    """Save raw arrays, metadata/provenance and deterministic-scale PNG/PDF figures."""
    import matplotlib
    import matplotlib.pyplot as plt
    arrays = raw_arrays(planes)
    manifest = {
        'schema': 'sonar3d.scientific_image.v1', 'source': source,
        'rendering': {**asdict(config), 'origin': 'upper', 'interpolation': 'nearest',
                      'azimuth_positive': 'left', 'elevation_positive': 'up',
                      'angular_extent': 'half-bin edges around vendor pixel centres'},
        'products': {product: plane.metadata for product, plane in planes.items()},
        'arrays': {name: {'shape': list(data.shape), 'dtype': data.dtype.str,
                          'sha256': hashlib.sha256(data.tobytes()).hexdigest()}
                   for name, data in arrays.items()},
        'units': {'range_m': 'm (slant range)', 'azimuth_deg': 'degrees, positive left',
                  'elevation_deg': 'degrees, positive up',
                  'signal_code': 'vendor logarithmic code, 0 means no return',
                  'signal_strength_relative': 'vendor-relative units; not calibrated backscatter',
                  'shaded_code': 'qualitative vendor rendering; not a distance measurement'},
        'signal_conversion': '0 -> 0; otherwise round(30 * 10 ** (signal_code / 100))',
        'software': {'python': platform.python_version(), 'numpy': np.__version__,
                     'matplotlib': matplotlib.__version__,
                     'analysis_source_sha256': sha256_file(__file__)},
        'statistics': {'range': statistics(arrays['range_m'], arrays['valid_range'],
                                           config.range_limits)},
    }
    from matplotlib import font_manager
    manifest['software']['font_sha256'] = sha256_file(font_manager.findfont('DejaVu Sans'))
    if 'signal_code' in arrays:
        name = 'signal_code' if config.signal_scale == 'encoded' else 'signal_strength_relative'
        manifest['statistics']['signal'] = statistics(
            arrays[name], arrays['valid_signal'], config.signal_limits)
    # Validate metadata serialization before making a bundle; never overwrite a run.
    encoded = json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + '\n'
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(directory / 'data.npz', **arrays)
    (directory / 'metadata.json').write_text(encoded)
    figure = ScientificFigure(config)
    try:
        figure.update(planes)
        figure.figure.savefig(directory / 'figure.png', dpi=config.dpi,
                              metadata={'Software': 'sonar3d_image_viewer'})
        figure.figure.savefig(directory / 'figure.pdf', metadata={
            'Creator': 'sonar3d_image_viewer', 'CreationDate': None, 'ModDate': None})
    finally:
        plt.close(figure.figure)
    return manifest
