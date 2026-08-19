"""Unit tests for bounded Sonar 3D-15 configuration through wlsonar."""

import pytest

from sonar3d.http_api import configure_sonar


class _SonarClient:
    """Capture wlsonar-style calls without contacting sonar hardware."""

    def __init__(self, ip_address, timeout):
        """Record constructor settings."""
        self.ip_address = ip_address
        self.timeout = timeout
        self.calls = []

    def set_speed_of_sound(self, speed):
        """Record speed and the effective request timeout."""
        self.calls.append(('speed_of_sound', speed, self.timeout))

    def set_acoustics_enabled(self, enabled):
        """Record acoustics and the effective request timeout."""
        self.calls.append(('acoustics', enabled, self.timeout))

    def set_udp_multicast(self):
        """Record multicast and the effective request timeout."""
        self.calls.append(('multicast', None, self.timeout))


class _ClientFactory:
    """Retain the client returned to configure_sonar."""

    def __init__(self):
        """Create a factory with no client yet."""
        self.client = None

    def __call__(self, ip_address, timeout):
        """Create and retain a fake client."""
        self.client = _SonarClient(ip_address, timeout)
        return self.client


def test_default_configuration_is_bounded():
    factory = _ClientFactory()

    applied = configure_sonar(
        '192.0.2.1',
        timeout=2.5,
        client_factory=factory,
    )

    assert applied == ['acoustics', 'multicast']
    assert factory.client.ip_address == '192.0.2.1'
    assert factory.client.calls == [
        ('acoustics', True, 2.5),
        ('multicast', None, 2.5),
    ]


def test_speed_configuration_allows_slow_device_update():
    factory = _ClientFactory()

    applied = configure_sonar(
        '192.0.2.1',
        speed_of_sound=1491,
        timeout=2.5,
        client_factory=factory,
    )

    assert applied == ['speed_of_sound', 'acoustics', 'multicast']
    assert factory.client.calls == [
        ('speed_of_sound', 1491.0, 30.0),
        ('acoustics', True, 2.5),
        ('multicast', None, 2.5),
    ]


@pytest.mark.parametrize('speed', [-1, 999, 2001])
def test_invalid_speed_of_sound_is_rejected(speed):
    with pytest.raises(ValueError, match='between 1000 and 2000'):
        configure_sonar(
            '192.0.2.1',
            speed_of_sound=speed,
            client_factory=_ClientFactory(),
        )


def test_invalid_timeout_is_rejected_before_client_creation():
    factory = _ClientFactory()

    with pytest.raises(ValueError, match='greater than zero'):
        configure_sonar('192.0.2.1', timeout=0.0, client_factory=factory)

    assert factory.client is None
