#!/usr/bin/env python3
# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""View live quantitative sonar images or export reproducible recording figures."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import tempfile
import time

from sonar_image_science import (
    ImageMatcher, Plane, PlotConfig, PRODUCTS, save_bundle, ScientificFigure, sha256_file, Shots,
)


def config_from_arguments(args, defaults=None):
    """Use explicit fixed scales; rerendering a bundle reuses its saved settings."""
    settings = defaults or asdict(PlotConfig())
    scale = args.signal_scale or settings['signal_scale']
    signal_limits = args.signal_limits
    if signal_limits is None:
        signal_limits = settings['signal_limits'] if scale == settings['signal_scale'] else (
            (0, 255) if scale == 'encoded' else (0, 10644))
    return PlotConfig(tuple(args.range_limits or settings['range_limits']),
                      tuple(signal_limits), scale, args.colormap or settings['colormap'],
                      args.dpi if args.dpi is not None else settings['dpi'])


def read_bag_frame(path, namespace, index, sequence):
    """Select a shot only from raw images paired with exact acquisition metadata."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from sonar3d.msg import ImageMetadata
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    matcher, shots = ImageMatcher(), Shots()
    prefix = '/' + namespace.strip('/') + '/' if namespace.strip('/') else '/'
    selected = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        product = topic.removeprefix(prefix)
        if product in PRODUCTS.values() and topic == prefix + product:
            plane = matcher.add_image(product, deserialize_message(serialized, Image))
        elif topic == prefix + 'image_metadata':
            plane = matcher.add_metadata(deserialize_message(serialized, ImageMetadata))
        else:
            continue
        if plane is None:
            continue
        frame = shots.add(plane)
        if frame['index'] is None:
            continue
        matches = (frame['index'] == index if sequence is None else
                   frame['planes']['range_image'].metadata['sequence_id'] == sequence)
        if matches:
            selected = frame
        elif selected is not None and frame['index'] > selected['index']:
            break
    if selected is None:
        raise ValueError('No selected range image with matching image_metadata. '
                         'Use a bag produced by this driver or convert the original .sonar file.')
    return selected


def load_bundle(path):
    """Verify array content hashes before using a prior scientific data bundle."""
    import hashlib
    import numpy as np
    path = Path(path)
    manifest = json.loads((path / 'metadata.json').read_text())
    if manifest.get('schema') != 'sonar3d.scientific_image.v1':
        raise ValueError('unsupported scientific image bundle schema')
    with np.load(path / 'data.npz', allow_pickle=False) as stored:
        arrays = {name: stored[name] for name in stored.files}
    for name, descriptor in manifest['arrays'].items():
        data = arrays[name]
        if (list(data.shape) != descriptor['shape'] or data.dtype.str != descriptor['dtype'] or
                hashlib.sha256(data.tobytes()).hexdigest() != descriptor['sha256']):
            raise ValueError(f'array checksum/shape/type mismatch: {name}')
    names = {'range_image': 'range_m', 'intensity_image': 'signal_code',
             'shaded_image': 'shaded_code'}
    planes = {product: Plane(product, arrays[names[product]], metadata)
              for product, metadata in manifest['products'].items()}
    return planes, manifest


def input_provenance(path, kind):
    """Identify a source independently of its human-readable filename."""
    path = Path(path).resolve()
    result = {'kind': kind, 'path': str(path)}
    if path.is_file():
        result['sha256'] = sha256_file(path)
    else:
        result['files'] = {str(item.relative_to(path)): sha256_file(item)
                           for item in sorted(path.rglob('*')) if item.is_file()}
    return result


def offline(args):
    """Export a selected recorded shot or faithfully rerender a saved data bundle."""
    import matplotlib
    matplotlib.use('Agg')
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix('sonar3d'))
    if args.bundle:
        planes, previous = load_bundle(args.bundle)
        config = config_from_arguments(args, previous['rendering'])
        source = {**input_provenance(args.bundle, 'scientific_bundle'),
                  'original_source': previous['source']}
    else:
        config = config_from_arguments(args)
        if args.file:
            source = input_provenance(args.file, 'sonar_recording')
            binary = prefix / 'lib/sonar3d/sonar_replay'
            source['converter'] = {'executable_sha256': sha256_file(binary),
                                   'libraries': {name: sha256_file(prefix / 'lib' / name)
                                                 for name in ('libsonar3d_protocol.so',
                                                              'libsonar3d_ros.so')}}
            # Reuse the native decoder/conversions; no independent Python RIP parser.
            with tempfile.TemporaryDirectory(prefix='sonar3d-scientific-') as temporary:
                bag = Path(temporary) / 'bag'
                subprocess.run([str(binary), '--file', str(Path(args.file).resolve()),
                                '--validate-only'], check=True)
                source['supported_product_validation'] = 'passed'
                subprocess.run([str(binary), '--file', str(Path(args.file).resolve()),
                                '--output', str(bag), '--ros-args', '-r',
                                '__ns:=/' + args.namespace.strip('/')], check=True)
                frame = read_bag_frame(bag, args.namespace, args.frame_index, args.sequence_id)
        else:
            source = input_provenance(args.bag, 'rosbag2')
            frame = read_bag_frame(args.bag, args.namespace, args.frame_index, args.sequence_id)
        source['range_frame_index'] = frame['index']
        planes = frame['planes']
    meta = planes['range_image'].metadata
    output = args.output or f"sonar-shot-{meta['sequence_id']}"
    save_bundle(output, planes, config, source)
    print(f'Saved scientific image bundle: {Path(output).resolve()}')


def live(args):
    """View matched live images with fixed scales, pause and snapshot controls."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from sonar3d.msg import ImageMetadata
    rclpy.init()
    node = rclpy.create_node('scientific_image_viewer', namespace=args.namespace)
    matcher, shots = ImageMatcher(), Shots()

    def accept(plane):
        if plane is not None:
            shots.add(plane)

    def image(product, message):
        try:
            accept(matcher.add_image(product, message))
        except ValueError as error:
            node.get_logger().warning(f'Image rejected: {error}')

    def metadata(message):
        try:
            accept(matcher.add_metadata(message))
        except ValueError as error:
            node.get_logger().warning(f'Image metadata rejected: {error}')

    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(
        Image, product, lambda msg, product=product: image(product, msg), qos)
        for product in PRODUCTS.values()]
    subscriptions.append(node.create_subscription(ImageMetadata, 'image_metadata', metadata, qos))
    config = config_from_arguments(args)
    source = {'kind': 'live_ros', 'namespace': '/' + args.namespace.strip('/')}
    if args.source_recording:
        source.update(input_provenance(args.source_recording, 'recording_replay_ros'))
    plot = ScientificFigure(config)
    plot.figure.canvas.manager.set_window_title('Sonar 3D-15 · Quantitative images')
    status = plot.figure.text(0.065, 0.04, 'Waiting for raw images and exact metadata…',
                              fontsize=9)
    pause = Button(plot.figure.add_axes((0.69, 0.04, 0.11, 0.055)), 'Pause')
    capture = Button(plot.figure.add_axes((0.82, 0.04, 0.14, 0.055)), 'Save frame')
    state = {'paused': False, 'frame': None, 'signature': None}

    def toggle(_):
        state['paused'] = not state['paused']
        pause.label.set_text('Resume' if state['paused'] else 'Pause')
        plot.figure.canvas.draw_idle()

    def save(_):
        if state['frame'] is None:
            return
        planes = state['frame']['planes']
        meta = planes['range_image'].metadata
        name = (f"shot-{meta['sequence_id']}-{meta['sensor_stamp_seconds']}-"
                f"{meta['sensor_stamp_nanoseconds']:09d}")
        directory = Path(args.output or 'sonar-image-captures') / name
        try:
            provenance = {**source, 'range_frame_index_observed': state['frame']['index']}
            save_bundle(directory, planes, config, provenance)
            status.set_text(f'Saved {directory.name}')
            node.get_logger().info(f'Saved scientific image bundle: {directory.resolve()}')
        except (OSError, ValueError) as error:
            status.set_text(f'Save failed: {error}')
        plot.figure.canvas.draw_idle()

    pause.on_clicked(toggle)
    capture.on_clicked(save)
    plt.show(block=False)
    rendered = 0.0
    try:
        while rclpy.ok() and plt.fignum_exists(plot.figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            for _ in range(31):
                rclpy.spin_once(node, timeout_sec=0.0)
            frame = shots.latest()
            if (frame is not None and not state['paused'] and
                    time.monotonic() - rendered >= 1 / args.refresh_hz):
                signature = (frame['index'], tuple(frame['planes']))
                if signature != state['signature']:
                    try:
                        plot.update(frame['planes'])
                        # Freeze a copy so saving/pausing cannot later mix changed products.
                        state['frame'] = {**frame, 'planes': dict(frame['planes'])}
                        state['signature'] = signature
                        shot_id = frame['planes']['range_image'].metadata['sequence_id']
                        status.set_text(f'Shot {shot_id} · fixed scales · '
                                        'Pause to inspect, Save frame to export')
                        rendered = time.monotonic()
                    except ValueError as error:
                        status.set_text(f'Frame rejected: {error}')
            plot.figure.canvas.flush_events()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close(plot.figure)


def main():
    """Dispatch live viewing, recording export or data-bundle rerendering."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--file', help='Original .sonar recording (native conversion)')
    modes.add_argument('--bag', help='rosbag2 with images and image_metadata')
    modes.add_argument('--bundle', help='Existing scientific bundle to verify and rerender')
    parser.add_argument('--namespace', default='sonar3d')
    parser.add_argument('--output', help='New offline bundle or directory for live snapshots')
    parser.add_argument('--source-recording',
                        help='Attach this recording SHA-256 to live replay snapshots')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--frame-index', type=int, default=0,
                           help='Zero-based range frame in bag read order')
    selection.add_argument('--sequence-id', type=int, help='Select source shot ID')
    parser.add_argument('--range-limits', nargs=2, type=float, metavar=('MIN_M', 'MAX_M'))
    parser.add_argument('--signal-limits', nargs=2, type=float, metavar=('MIN', 'MAX'))
    parser.add_argument('--signal-scale', choices=('encoded', 'linear'))
    parser.add_argument('--colormap', choices=('cividis', 'viridis', 'magma'))
    parser.add_argument('--dpi', type=int)
    parser.add_argument('--refresh-hz', type=float, default=5.0,
                        help='Live display refresh limit; no frame averaging')
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args()[1:])
    if args.frame_index < 0:
        parser.error('--frame-index must be nonnegative')
    if not 0 < args.refresh_hz <= 60:
        parser.error('--refresh-hz must be in (0, 60]')
    try:
        if args.file or args.bag or args.bundle:
            offline(args)
        else:
            live(args)
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
