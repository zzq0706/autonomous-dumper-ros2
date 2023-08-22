import glob
import os

from setuptools import setup

package_name = 'lidar_slam_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [
                                                'rviz/nav2_unity.rviz',
                                                'launch/lidar_slam_example.launch.py',
                                                'config/nav2_params.yaml',
                                                'config/mapper_params_online_async.yaml'
                                                ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unity Robotics',
    maintainer_email='unity-robotics@unity3d.com',
    description='Unity Robotics Nav2 SLAM Example',
    license='Apache 2.0',
    tests_require=['pytest']
)
