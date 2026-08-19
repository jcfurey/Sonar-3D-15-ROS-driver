"""Bounded HTTP configuration calls for the Sonar 3D-15 integration API."""

import requests


def configure_sonar(ip_address, speed_of_sound=0, timeout=5.0, session=None):
    """
    Enable acoustics and multicast, optionally setting speed of sound.

    Return the names of settings successfully applied. Every request has a
    timeout so an absent sonar cannot wedge ROS launch indefinitely.
    """
    if speed_of_sound and not 1000 <= speed_of_sound <= 2000:
        raise ValueError('speed_of_sound must be zero or between 1000 and 2000 m/s')
    client = session if session is not None else requests
    base_url = f'http://{ip_address}/api/v1/integration'
    applied = []
    if speed_of_sound:
        response = client.post(
            f'{base_url}/acoustics/speed_of_sound',
            json=speed_of_sound,
            timeout=max(timeout, 30.0),
        )
        response.raise_for_status()
        applied.append('speed_of_sound')

    response = client.post(
        f'{base_url}/acoustics/enabled',
        json=True,
        timeout=timeout,
    )
    response.raise_for_status()
    applied.append('acoustics')

    response = client.post(
        f'{base_url}/udp',
        json={
            'mode': 'multicast',
            'unicast_destination_ip': '',
            'unicast_destination_port': 0,
        },
        timeout=timeout,
    )
    response.raise_for_status()
    applied.append('multicast')
    return applied
