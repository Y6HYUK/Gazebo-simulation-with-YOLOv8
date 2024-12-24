from setuptools import find_packages, setup
import os

package_name = 'gazebo_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add sdf file
        (os.path.join('share', package_name, 'worlds'), ['gazebo_yolo/worlds/custom_gas_station.world']),
    ],
    install_requires=['setuptools', 'PyQt5', 'opencv-python'],
    zip_safe=True,
    maintainer='yjh',
    maintainer_email='y6hyuk@naver.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camera_viewer_node = gazebo_yolo.camera_viewer_node:main'
        ],
    },
)
