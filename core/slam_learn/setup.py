import os
from glob import glob

from setuptools import setup

package_name = 'slam_learn'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinyicheng',
    maintainer_email='jinyicheng@todo.todo',
    description='手写最简 2D 激光 SLAM（学习用）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_slam1 = slam_learn.simple_slam1:main',
        ],
    },
)
