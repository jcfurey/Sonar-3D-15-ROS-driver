"""Unit tests for bounded Sonar 3D-15 HTTP configuration."""

import pytest

from sonar3d.http_api import configure_sonar


class _Response:
    """Minimal successful requests response test double."""

    def raise_for_status(self):
        """Represent a successful HTTP response."""


class _Session:
    """Capture HTTP calls without contacting sonar hardware."""

    def __init__(self):
        """Create an empty call list."""
        self.calls = []

    def post(self, url, **kwargs):
        """Record one POST and return a successful response."""
        self.calls.append((url, kwargs))
        return _Response()


def test_default_configuration_is_bounded():
    session = _Session()

    applied = configure_sonar('192.0.2.1', timeout=2.5, session=session)

    assert applied == ['acoustics', 'multicast']
    assert [call[1]['timeout'] for call in session.calls] == [2.5, 2.5]
    assert session.calls[0][1]['json'] is True
    assert session.calls[1][1]['json']['mode'] == 'multicast'


def test_speed_configuration_allows_slow_device_update():
    session = _Session()

    applied = configure_sonar(
        '192.0.2.1',
        speed_of_sound=1491,
        timeout=2.5,
        session=session,
    )

    assert applied == ['speed_of_sound', 'acoustics', 'multicast']
    assert session.calls[0][1] == {'json': 1491, 'timeout': 30.0}


@pytest.mark.parametrize('speed', [-1, 999, 2001])
def test_invalid_speed_of_sound_is_rejected(speed):
    with pytest.raises(ValueError, match='between 1000 and 2000'):
        configure_sonar('192.0.2.1', speed_of_sound=speed, session=_Session())
