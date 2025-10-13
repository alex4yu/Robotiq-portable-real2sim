from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'start_imu',
            default_value='true',
            description='Start IMU publisher'
        ),
        DeclareLaunchArgument(
            'start_joystick',
            default_value='true', 
            description='Start joystick teleop'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU data topic'
        ),
        DeclareLaunchArgument(
            'temp_topic',
            default_value='/imu/temperature',
            description='Temperature topic'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='IMU frame ID'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50',
            description='IMU publish rate (Hz)'
        ),
        
        # IMU Publisher Node
        Node(
            package='m5_imu_pro_ros2',
            executable='imu_publisher',
            name='bmi270_imu_publisher',
            parameters=[{
                'i2c_bus': 1,
                'address': 0x68,
                'accel_scale': 4,
                'gyro_scale': 2000,
                'imu_topic': LaunchConfiguration('imu_topic'),
                'temp_topic': LaunchConfiguration('temp_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'debug': False
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_imu'))
        ),
        
        # Joystick Teleop Process
        ExecuteProcess(
            cmd=['python3', '/path/to/joystick_combined_control.py'],
            name='joystick_teleop',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_joystick'))
        ),
    ])
