from setuptools import setup

package_name = 'armpi_grasp_planner'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jericho Cheng',
    maintainer_email='maintainer@example.invalid',
    description=(
        '5-DoF constrained grasp candidate search and MoveIt 2 screening '
        'for ArmPi Ultra'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'grasp_candidate_check = '
                'armpi_grasp_planner.cli:main'
            ),
        ],
    },
)
