from glob import glob
import os

from setuptools import setup

package_name = 'armpi_safety'

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
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.*')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jericho Cheng',
    maintainer_email='maintainer@example.invalid',
    description=(
        'Conservative state watchdog and trajectory veto for ArmPi Ultra'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor = armpi_safety.node:main',
        ],
    },
)
