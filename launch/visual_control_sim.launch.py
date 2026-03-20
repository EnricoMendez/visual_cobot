from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    trajectory_action = LaunchConfiguration('trajectory_action')
    home_joints = LaunchConfiguration('home_joints')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_gazebo'),
                'launch',
                'lite6_beside_table_gazebo.launch.py',
            ])
        ),
    )

    visual_control_sim_node = Node(
        package='visual_cobot',
        executable='visual_control_sim',
        name='visual_control_sim',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'trajectory_action': trajectory_action,
            'home_joints': home_joints,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'trajectory_action',
            default_value='/lite6_traj_controller/follow_joint_trajectory',
        ),
        DeclareLaunchArgument(
            'home_joints',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        ),
        gazebo_launch,
        visual_control_sim_node,
    ])
