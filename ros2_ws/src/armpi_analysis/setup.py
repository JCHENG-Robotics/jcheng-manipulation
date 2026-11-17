"""Build the armpi_analysis ROS 2 package."""

from glob import glob
import os

from setuptools import setup

package_name = 'armpi_analysis'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jericho Cheng',
    maintainer_email='maintainer@example.invalid',
    description=(
        'Dependency-free error-budget, paired-trial and host benchmark tools'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'error_budget = armpi_analysis.error_budget:main',
            'paired_trials = armpi_analysis.paired_trials:main',
            'host_benchmark = armpi_analysis.host_benchmark:main',
        ],
    },
)
