from glob import glob

from setuptools import find_packages, setup

package_name = 'rescue_team'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='jsolevag@gmail.com',
    description='Two-robot search and rescue for DAT160 (group 11)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_handler = rescue_team.nodes.robot_handler:main',
            'marker_detector = rescue_team.nodes.marker_detector:main',
            'leader = rescue_team.nodes.leader:main',
        ],
    },
)
