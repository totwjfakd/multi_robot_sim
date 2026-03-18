from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fleet_nav'

def glob_recursive(pattern):
    """Recursively glob files."""
    results = []
    for match in glob(pattern, recursive=True):
        results.append(match)
    return results

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
        *[
            (os.path.join('share', package_name, os.path.dirname(f)),
             [f])
            for f in glob_recursive('models/**/*')
            if os.path.isfile(f)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Fleet navigation with GT pose, Route Server, and RPP',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gt_pose_bridge = fleet_nav.gt_pose_bridge:main',
            'fleet_commander = fleet_nav.fleet_commander:main',
            'multi_robot_controller = fleet_nav.multi_robot_controller:main',
            'show_origins = fleet_nav.show_origins:main',
        ],
    },
)
