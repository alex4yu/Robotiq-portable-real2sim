from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'm5_imu_pro_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='BMI270 IMU publisher for Robotiq gripper system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = m5_imu_pro_ros2.imu_pro_read_ros_publish:main',
        ],
    },
)