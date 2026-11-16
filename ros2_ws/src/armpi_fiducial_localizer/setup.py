from glob import glob
import os

from setuptools import setup

package_name = 'armpi_fiducial_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jericho Cheng',
    maintainer_email='maintainer@example.invalid',
    description='AprilTag/ArUco target localization for ArmPi Ultra',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducial_localizer = armpi_fiducial_localizer.node:main',
            (
                'synthetic_marker_publisher = '
                'armpi_fiducial_localizer.synthetic_publisher:main'
            ),
            (
                'fiducial_synthetic_check = '
                'armpi_fiducial_localizer.cli:main'
            ),
        ],
    },
)
