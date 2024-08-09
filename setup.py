from setuptools import setup
import os
from glob import glob

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'weights'), glob('scripts/weights/**/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team SOBITS',
    maintainer_email='choi.6f@gmail.com',
    description='YOLOv8 ROS 2 package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ros = yolov8_ros.detect_ros:main',
        ],
    },
)
