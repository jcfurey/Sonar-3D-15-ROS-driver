from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sonar3d'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Water Linked',
    maintainer_email='support@waterlinked.com',
    description='ROS 2 driver and playback tools for the Water Linked Sonar 3D-15',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_publisher = sonar3d.multicast_listener:main',
            'sonar_to_bag = sonar3d.sonar_to_bag:main',
        ],
    },
)
