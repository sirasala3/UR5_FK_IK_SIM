from setuptools import setup
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur5_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur5_full.launch.py', 'launch/ur5_demo.launch.py', 'launch/ur5_rviz_view.launch.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shiva',
    maintainer_email='shiva@example.com',
    description='UR5 ROS2 motion package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_controller = ur5_motion.main:main',
        ],
    },
)

