from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device path, for example /dev/video0',
        ),
        usb_cam_node,
        gesture_recognition_node,
    ])
