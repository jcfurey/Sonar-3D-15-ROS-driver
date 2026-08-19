"""Bounded Sonar 3D-15 configuration through the official wlsonar client."""

import wlsonar


def configure_sonar(
        ip_address, speed_of_sound=0.0, timeout=5.0, client_factory=None):
    """Enable acoustics and multicast, optionally setting speed of sound."""
    if timeout <= 0.0:
        raise ValueError('timeout must be greater than zero')
    if speed_of_sound and not 1000 <= speed_of_sound <= 2000:
        raise ValueError('speed_of_sound must be zero or between 1000 and 2000 m/s')

    factory = client_factory if client_factory is not None else wlsonar.Sonar3D
    sonar = factory(ip_address, timeout=float(timeout))
    applied = []

    if speed_of_sound:
        ordinary_timeout = getattr(sonar, 'timeout', None)
        if ordinary_timeout is not None:
            sonar.timeout = max(float(timeout), 30.0)
        try:
            sonar.set_speed_of_sound(float(speed_of_sound))
        finally:
            if ordinary_timeout is not None:
                sonar.timeout = ordinary_timeout
        applied.append('speed_of_sound')

    sonar.set_acoustics_enabled(True)
    applied.append('acoustics')
    sonar.set_udp_multicast()
    applied.append('multicast')
    return applied
