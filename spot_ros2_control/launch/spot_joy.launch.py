import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'spot_ros2_control'

    # Declare the spot_name launch argument
    spot_name_arg = DeclareLaunchArgument(
        'spot_name',
        default_value='',
        description='Namespace for Spot robot nodes and topics (e.g., spot1)'
    )
    spot_name_config = LaunchConfiguration('spot_name')

    # Get config file paths
    teleop_joy_config_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'logitech_spot_teleop.yaml'
    )

    joy_service_mappings_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'joy_service_mapping.yaml'
    )

    joy_body_pose_config_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'joy_body_control_config.yaml' # Config for joy_body_pose_controller
    )

    # Config file for the new translator node
    pose_to_robot_command_config_filepath = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'pose_to_robot_command_config.yaml' # Config for pose_to_robot_command_translator
    )

    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=spot_name_config,
        parameters=[{
            'dev': '/dev/input/js0', # Adjust if your joystick is different
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=spot_name_config,
        parameters=[teleop_joy_config_filepath],
    )

    joy_service_caller_node = Node(
        package=package_name,
        executable='joy_service_caller',
        name='joy_service_caller',
        namespace=spot_name_config,
        output='screen',
        parameters=[{
            'mappings_file_path': joy_service_mappings_filepath,
            'spot_name': spot_name_config
        }]
    )

    joy_body_pose_controller_node = Node(
        package=package_name,
        executable='joy_body_pose_controller',
        name='joy_body_pose_controller',
        namespace=spot_name_config,
        output='screen',
        parameters=[{
            "config_file_path": joy_body_pose_config_filepath,
            "spot_name": spot_name_config  # <<< ADDED spot_name parameter
        }]
    )

    pose_to_robot_command_node = Node(
        package=package_name,
        executable='pose_to_robot_command_translator',
        name='pose_to_robot_command_translator',
        namespace=spot_name_config,
        output='screen',
        parameters=[
            pose_to_robot_command_config_filepath,
            {'spot_name': spot_name_config}
        ]
    )

    return LaunchDescription([
        spot_name_arg,
        joy_node,
        teleop_twist_joy_node,
        joy_service_caller_node,
        joy_body_pose_controller_node,
        pose_to_robot_command_node
    ])
