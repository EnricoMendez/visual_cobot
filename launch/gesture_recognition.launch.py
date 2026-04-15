from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Camera device path, for example /dev/video0',
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('visual_cobot'),
                'config',
                'rviz_config.rviz',
            ]
        ),
        description='RViz config file path',
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'pixel_format': 'yuyv',
            'image_width': 640,
            'image_height': 480,
        }],
    )

    gesture_recognition_node = Node(
        package='visual_cobot',
        executable='gesture_recognition',
        name='gesture_recognition',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw',
            'annotated_image_topic': '/hand/annotated_image',
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
    )

    return LaunchDescription([
        video_device_arg,
        rviz_config_file_arg,
        usb_cam_node,
        gesture_recognition_node,
        rviz_node,
    ])
