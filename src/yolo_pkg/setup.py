from setuptools import find_packages, setup

package_name = 'yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoniofaical',
    maintainer_email='antoniofaical@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = yolo_pkg.camera_node:main',
            'yolo_node = yolo_pkg.yolo_node:main',
            'yolo_visualizer = yolo_pkg.yolo_visualizer_node:main',
            'yolo_listener = yolo_pkg.yolo_listener_node:main',
        ],
    },
)
