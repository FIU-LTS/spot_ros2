import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Substitute 'your_actual_package_name' with the name of your ROS 2 package
    package_name = 'spot_ros2_control'

    # Path to teleop_twist_joy configuration (if still used)
    teleop_joy_config_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'logitech_spot_teleop.yaml' # Assuming this is your existing teleop config
    )

    # Path to the new joy_to_service_mappings configuration
    joy_to_service_mappings_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config', # Assuming you place it in a 'config' subfolder within share
        'joy_service_mapping.yaml' # Renamed config file
    )

    # Node for reading joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node', # Standard name for the joy driver node
        parameters=[{
            'dev': '/dev/input/js0', # Verify your joystick device
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Node for converting joystick to cmd_vel (if still needed)
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node', # Common name for this node
        parameters=[teleop_joy_config_filepath],
    )

    # C++ Node for mapping joystick buttons to service calls
    joy_service_caller_node = Node(
        package=package_name,
        executable='joy_service_caller', # Renamed executable
        name='joy_service_caller', # Renamed runtime node name
        output='screen',
        parameters=[{
            'mappings_file_path': joy_to_service_mappings_filepath
        }]
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node, # Uncomment if you still need cmd_vel control from joystick
        joy_service_caller_node
    ])