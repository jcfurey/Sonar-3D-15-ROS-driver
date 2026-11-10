#!/usr/bin/env python3
# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Record a ROS bag together with the live driver's byte-exact .sonar stream."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


class RecordingInterrupted(Exception):
    """User requested shutdown while a service was in progress."""


def recording_state(status):
    return {key: getattr(status, key) for key in (
        'active', 'path', 'packets', 'bytes', 'queued_bytes', 'dropped_packets', 'error')}


def bag_command(output, recorder_name, driver_node, arguments):
    protected = {'-o', '--output', '--node-name', '--start-paused', '--snapshot-mode'}
    if any(argument.split('=', 1)[0] in protected or
           (argument.startswith('-o') and not argument.startswith('--'))
           for argument in arguments):
        raise ValueError(
            'output, recorder name, and paused startup are controlled by sonar_record_bag')
    if not arguments:
        namespace = driver_node.rstrip('/').rsplit('/', 1)[0]
        arguments = ['--topics', *[
            namespace + '/' + suffix for suffix in (
                'raw_packet', 'points', 'range_image', 'intensity_image',
                'shaded_image', 'image_metadata', 'imu/data_raw')], '/tf_static', '/diagnostics']
    return ['ros2', 'bag', 'record', '--output', str(output), '--storage', 'mcap',
            '--node-name', recorder_name, '--start-paused', '--disable-keyboard-controls',
            *arguments]


class RecordingSession:
    """Coordinate a paused recorder with the driver; never stop another capture."""

    def __init__(self, args):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rosbag2_interfaces.srv import IsPaused, Pause, Resume
        from sonar3d.srv import StartRecording, StopRecording, GetRecordingStatus

        self.rclpy = rclpy
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('sonar_record_bag_controller_' + str(os.getpid()))
        self.args = args
        self.process = None
        self.started = False
        self.start_requested = False
        self.stop_requested = 0
        self.closing = False
        self.report_enabled = False
        self.output = args.output.expanduser().resolve()
        self.native_path = self.output / 'sonar.sonar'
        self.report = {'driver_node': args.driver_node, 'native_file': str(self.native_path),
                       'phase': 'starting', 'started_utc': datetime.now(timezone.utc).isoformat()}
        recorder = '/' + args.recorder_name
        driver = args.driver_node.rstrip('/')
        self.start_client = self.node.create_client(StartRecording, driver + '/start_recording')
        self.stop_client = self.node.create_client(StopRecording, driver + '/stop_recording')
        self.status_client = self.node.create_client(
            GetRecordingStatus, driver + '/get_recording_status')
        self.paused_client = self.node.create_client(IsPaused, recorder + '/is_paused')
        self.pause_client = self.node.create_client(Pause, recorder + '/pause')
        self.resume_client = self.node.create_client(Resume, recorder + '/resume')

    def call(self, client, request, timeout=None):
        if self.stop_requested and not self.closing:
            raise RecordingInterrupted('recording startup interrupted')
        deadline = time.monotonic() + (self.args.timeout if timeout is None else timeout)
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise RuntimeError('service unavailable: ' + client.srv_name)
            if client in (self.paused_client, self.pause_client, self.resume_client) and \
                    self.process and self.process.poll() is not None:
                raise RuntimeError('bag recorder exited before ' + client.srv_name + ' was ready')
            if self.stop_requested and not self.closing:
                raise RecordingInterrupted('recording interrupted while waiting for service')
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError('service timed out: ' + client.srv_name)
            if self.stop_requested and not self.closing:
                raise RecordingInterrupted('recording interrupted while waiting for service')
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        result = future.result()
        if result is None:
            raise RuntimeError('empty service result: ' + client.srv_name)
        return result

    def wait_for_bag_output(self):
        # rosbag services can be discovered before record() opens the writer.
        deadline = time.monotonic() + self.args.timeout
        while True:
            if self.process.poll() is not None:
                raise RuntimeError('bag recorder exited before opening its output')
            if self.stop_requested:
                raise RecordingInterrupted('recording startup interrupted')
            if self.output.is_dir() and any(path.is_file() for path in self.output.iterdir()):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('bag recorder did not create its output files')
            self.rclpy.spin_once(self.node, timeout_sec=.05)

    def write_report(self):
        if self.report_enabled and self.output.is_dir():
            temporary = self.output / '.native_recording.json.tmp'
            temporary.write_text(json.dumps(self.report, indent=2) + '\n')
            temporary.replace(self.output / 'native_recording.json')

    def run(self):
        from rosbag2_interfaces.srv import IsPaused, Resume
        from sonar3d.srv import StartRecording, GetRecordingStatus

        if self.output.exists():
            raise ValueError('bag output must not already exist: ' + str(self.output))
        if not self.output.parent.is_dir():
            raise ValueError('bag parent directory must exist: ' + str(self.output.parent))
        command = bag_command(self.output, self.args.recorder_name, self.args.driver_node,
                              self.args.bag_arguments)
        self.report['bag_command'] = command
        self.process = subprocess.Popen(command, start_new_session=True)
        paused = self.call(self.paused_client, IsPaused.Request())
        if not paused.paused:
            raise RuntimeError('recorder must remain paused during startup')
        self.wait_for_bag_output()
        self.report_enabled = True
        request = StartRecording.Request()
        request.path = str(self.native_path)
        self.start_requested = True
        response = self.call(self.start_client, request)
        if not response.success:
            raise RuntimeError('native recording did not start: ' + response.message)
        self.started = True
        self.call(self.resume_client, Resume.Request())
        if self.call(self.paused_client, IsPaused.Request()).paused:
            raise RuntimeError('bag recorder remained paused')
        self.report['phase'] = 'recording'
        self.report['status'] = recording_state(response.status)
        self.write_report()
        print('RECORDING READY: bag and native capture at ' + str(self.output), flush=True)
        began = time.monotonic()
        next_status = began
        while not self.stop_requested and self.process.poll() is None:
            now = time.monotonic()
            if self.args.duration and now - began >= self.args.duration:
                break
            if now >= next_status:
                status = self.call(self.status_client, GetRecordingStatus.Request()).status
                self.report['status'] = recording_state(status)
                self.write_report()
                if status.path != str(self.native_path) or not status.active or status.error:
                    raise RuntimeError(
                        'native capture stopped or failed: ' + (status.error or status.path))
                next_status = now + 1
            self.rclpy.spin_once(self.node, timeout_sec=.1)
        if self.process.poll() is not None and not self.stop_requested:
            raise RuntimeError('bag recorder exited unexpectedly: ' + str(self.process.returncode))
        return 0

    def close(self):
        from rosbag2_interfaces.srv import Pause
        from sonar3d.srv import GetRecordingStatus, StopRecording

        self.closing = True
        error = None
        try:
            # Native capture brackets the bag: pre/post-roll covers service latency.
            if self.process and self.process.poll() is None:
                try:
                    self.call(self.pause_client, Pause.Request(), timeout=5)
                except Exception as exception:
                    error = str(exception)
            if self.start_requested:
                state = self.call(
                    self.status_client, GetRecordingStatus.Request(), timeout=5).status
                if state.path != str(self.native_path):
                    if self.started:
                        raise RuntimeError(
                            'capture ownership changed; refusing to stop ' + state.path)
                else:
                    response = self.call(self.stop_client, StopRecording.Request())
                    self.report['status'] = recording_state(response.status)
                    if not response.success:
                        raise RuntimeError(response.message)
        except Exception as exception:
            error = str(exception)
        finally:
            if self.process and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait()
                    error = error or 'bag recorder did not finalize within 15 seconds'
            if self.process and self.process.returncode not in (None, 0, -signal.SIGINT):
                error = error or 'bag recorder exited with status ' + str(self.process.returncode)
            self.report['phase'] = 'failed' if error or self.report.get('error') else 'complete'
            self.report['stopped_utc'] = datetime.now(timezone.utc).isoformat()
            if error:
                self.report['error'] = error
            self.write_report()
            self.node.destroy_node()
            self.rclpy.shutdown()
        if error:
            raise RuntimeError(error)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--driver-node', default='/sonar3d/sonar3d_driver')
    parser.add_argument('--recorder-name', default='sonar3d_bag_recorder_' + str(os.getpid()))
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--duration', type=float, default=0.,
                        help='wall seconds; zero runs until stopped')
    parser.add_argument('bag_arguments', nargs=argparse.REMAINDER,
                        help='arguments after -- are forwarded to ros2 bag record')
    args = parser.parse_args(argv)
    if args.bag_arguments[:1] == ['--']:
        args.bag_arguments = args.bag_arguments[1:]
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not args.driver_node.startswith('/') or not args.driver_node.strip('/'):
        parser.error('--driver-node must be an absolute node name')
    if not args.recorder_name or '/' in args.recorder_name:
        parser.error('--recorder-name must be a node basename')
    # Validate arguments before creating a node or child process.
    try:
        bag_command(args.output, args.recorder_name, args.driver_node, args.bag_arguments)
    except ValueError as error:
        parser.error(str(error))
    session = RecordingSession(args)
    previous = {}
    for number in (signal.SIGINT, signal.SIGTERM):
        previous[number] = signal.signal(
            number, lambda signum, _frame: setattr(session, 'stop_requested', signum))
    result = 0
    try:
        result = session.run()
    except RecordingInterrupted:
        pass
    except Exception as error:
        session.report['error'] = str(error)
        print('Recording failed: ' + str(error), file=sys.stderr)
        result = 1
    finally:
        try:
            session.close()
        except Exception as error:
            print('Recording shutdown failed: ' + str(error), file=sys.stderr)
            result = 1
        for number, handler in previous.items():
            signal.signal(number, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
