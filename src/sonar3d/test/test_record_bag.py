# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT
"""Check CLI ownership and cleanup without requiring a running ROS graph."""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from sonar_record_bag import bag_command, RecordingSession


def test_default_topics_follow_driver_namespace():
    command = bag_command(Path('/data/dive'), 'capture', '/rov/sonar/driver', [])
    assert command[:3] == ['ros2', 'bag', 'record']
    assert '--start-paused' in command
    assert '/rov/sonar/raw_packet' in command
    assert '/tf_static' in command
    assert '--storage' in command and 'mcap' in command


@pytest.mark.parametrize('flag', [
    '-o', '-oelsewhere', '--output=elsewhere', '--node-name', '--start-paused',
    '--snapshot-mode',
])
def test_bag_arguments_cannot_override_coordination(flag):
    with pytest.raises(ValueError):
        bag_command(Path('/data/dive'), 'capture', '/sonar/driver', [flag])


def test_explicit_bag_topics_are_preserved():
    arguments = ['--topics', '/camera/image_raw', '/sonar/raw_packet']
    assert bag_command(Path('/data/dive'), 'capture', '/sonar/driver', arguments)[-3:] == \
        arguments[-3:]


@pytest.fixture
def session(monkeypatch, tmp_path):
    # close() imports these request types lazily. No generated ROS types are
    # needed to test whether a failed start can stop somebody else's capture.
    for name, types in [('rosbag2_interfaces.srv', ['Pause']),
                        ('sonar3d.srv', ['GetRecordingStatus', 'StopRecording'])]:
        module = ModuleType(name)
        for service in types:
            setattr(module, service, SimpleNamespace(Request=lambda: object()))
        monkeypatch.setitem(sys.modules, name, module)
    instance = RecordingSession.__new__(RecordingSession)
    instance.process = None
    instance.started = False
    instance.start_requested = True
    instance.output = tmp_path
    instance.native_path = tmp_path / 'sonar.sonar'
    instance.report = {}
    instance.report_enabled = True
    instance.pause_client = object()
    instance.stop_client = object()
    instance.status_client = object()
    instance.node = SimpleNamespace(destroy_node=lambda: None)
    instance.rclpy = SimpleNamespace(shutdown=lambda: None)
    return instance


def status(path):
    return SimpleNamespace(active=True, path=str(path), packets=5, bytes=500,
                           queued_bytes=0, dropped_packets=0, error='')


def test_failed_start_does_not_stop_existing_capture(session):
    calls = []

    def call(client, _request, **_kwargs):
        calls.append(client)
        return SimpleNamespace(status=status('/other.sonar'))

    session.call = call
    session.close()
    assert calls == [session.status_client]


def test_lost_start_response_still_flushes_our_file(session):
    calls = []

    def call(client, _request, **_kwargs):
        calls.append(client)
        return SimpleNamespace(status=status(session.native_path), success=True)

    session.call = call
    session.close()
    assert calls == [session.status_client, session.stop_client]
    assert session.report['status']['packets'] == 5


def test_changed_ownership_reports_error_without_stopping_other_capture(session):
    session.started = True
    calls = []

    def call(client, _request, **_kwargs):
        calls.append(client)
        return SimpleNamespace(status=status('/other.sonar'))

    session.call = call
    with pytest.raises(RuntimeError, match='ownership changed'):
        session.close()
    assert calls == [session.status_client]
    assert session.report['phase'] == 'failed'


def test_refused_existing_output_cannot_overwrite_its_manifest(session):
    session.report_enabled = False
    session.start_requested = False
    manifest = session.output / 'native_recording.json'
    manifest.write_text('previous capture metadata')
    session.close()
    assert manifest.read_text() == 'previous capture metadata'


def test_services_can_be_ready_before_bag_storage_opens(session):
    session.args = SimpleNamespace(timeout=1.)
    session.process = SimpleNamespace(poll=lambda: None)
    session.stop_requested = 0
    spins = []

    def spin(_node, **_kwargs):
        spins.append(True)
        (session.output / 'bag_0.mcap').touch()

    session.rclpy.spin_once = spin
    session.wait_for_bag_output()
    assert len(spins) == 1
