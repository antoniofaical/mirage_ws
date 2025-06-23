from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'movement_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafaelromaquela',
    maintainer_email='rafaelromaquela@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_motion=movement_test_pkg.input_motion:main',
            'test_input_motion_apriltag_absolute_pos=movement_test_pkg.test_input_motion_apriltag_absolute_pos:main',
            'test_input_motion_apriltag_relative_pos=movement_test_pkg.test_input_motion_apriltag_relative_pos:main',
            'publisher_cartesian_pose = movement_test_pkg.publisher_cartesian_pose:main',
            'publisher_apriltag_transform_node = movement_test_pkg.publisher_apriltag_transform_node:main',
            'test_motion_pick_place=movement_test_pkg.test_motion_pick_place:main'
        ],
    },
)
