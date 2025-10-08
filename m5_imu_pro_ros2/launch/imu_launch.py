from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m5_imu_pro_ros2',
            executable='imu_publisher',
            name='bmi270_imu_publisher',
            parameters=[{
                'i2c_bus': 1,
                'address': 0x68,
                'accel_scale': 4,
                'gyro_scale': 2000,
                'imu_topic': '/imu/data',
                'temp_topic': '/imu/temperature',
                'frame_id': 'imu_link',
                'publish_rate': 50,
                'debug': False
            }],
            output='screen'
        )
    ])