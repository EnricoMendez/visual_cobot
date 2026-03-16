from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video6',
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
        usb_cam_node,
        gesture_recognition_node,
    ])
