from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltags_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

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
        'plot_april_frame=apriltags_pkg.test_plot_april_frame:main',
        'test_plot_camera_and_april=apriltags_pkg.test_plot_camera_and_april:main',
        'test_plot_camera_and_two_april=apriltags_pkg.test_plot_camera_and_two_april:main',
        'test_transform_frame=apriltags_pkg.test_transform_frame:main',
        'test_camera_april_robotarm=apriltags_pkg.test_camera_april_robotarm:main',
        'pose_estimation_eyetracker=apriltags_pkg.pose_estimation_eyetracker:main',
        'visual_interface=apriltags_pkg.visual_interface:main',
        'visual_interface_four_apriltags=apriltags_pkg.visual_interface_four_apriltags:main',
        'visual_interface_with_arrow=apriltags_pkg.visual_interface_with_arrow:main',
        'print_image_eyetracker=apriltags_pkg.print_image_eyetracker:main',
        'dummy_publisher_eyetracker=apriltags_pkg.dummy_publisher_eyetracker:main',
        'einstein_visual_interface=apriltags_pkg.einstein_visual_interface:main',
        'real_eyetracker_publisher=apriltags_pkg.real_eyetracker_publisher:main',
        'test_perspective_eyetracker=apriltags_pkg.test_perspective_eyetracker:main'
        ],
    },
)
