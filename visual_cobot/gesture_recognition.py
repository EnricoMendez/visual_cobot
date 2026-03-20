#!/usr/bin/env python3

import os

import cv2
import mediapipe as mp
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


class GestureRecognition(Node):
    def __init__(self):
        super().__init__('gesture_recognition')
        self.get_logger().info('Starting gesture_recognition.')

        # State
        self.image_received = False
        self.cv_image = None
        self.last_image_header = None
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('annotated_image_topic', '/hand/annotated_image')
        self.declare_parameter('rate_hz', 0.1)
        self.declare_parameter('model_path', '')
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.annotated_image_topic = str(self.get_parameter('annotated_image_topic').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        model_path_param = str(self.get_parameter('model_path').value)

        if model_path_param:
            self.model_path = model_path_param
        else:
            share_dir = get_package_share_directory('visual_cobot')
            self.model_path = os.path.join(share_dir, 'config/gesture_recognizer.task')

        if not os.path.exists(self.model_path):
            self.get_logger().warning(
                f'Gesture model not found at: {self.model_path}. '
                'Set parameter "model_path" with a valid .task file.'
            )

        # MediaPipe recognizer setup
        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        self.options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=VisionRunningMode.IMAGE,
        )
        self.recognizer = GestureRecognizer.create_from_options(self.options)

        # Subscriber
        self.create_subscription(Image, self.image_topic, self.camera_callback, 10)

        # Publishers
        self.gesture_pub = self.create_publisher(String, '/hand/gesture', 10)
        self.position_pub = self.create_publisher(String, '/hand/position', 10)
        self.x_pub = self.create_publisher(Float64, '/hand/x', 10)
        self.y_pub = self.create_publisher(Float64, '/hand/y', 10)
        self.annotated_image_pub = self.create_publisher(Image, self.annotated_image_topic, 10)

        # Main loop timer
        timer_period = 0.2
        self.create_timer(timer_period, self.process_frame)

    def camera_callback(self, msg: Image):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_image_header = msg.header
        self.image_received = True

    def draw_result(self, image, result):
        annotated_image = image.copy()
        image_height, image_width = annotated_image.shape[:2]

        for hand_idx, hand_landmarks in enumerate(result.hand_landmarks):
            points = []
            for landmark in hand_landmarks:
                x_px = int(landmark.x * image_width)
                y_px = int(landmark.y * image_height)
                points.append((x_px, y_px))

            for start_idx, end_idx in HAND_CONNECTIONS:
                cv2.line(
                    annotated_image,
                    points[start_idx],
                    points[end_idx],
                    (255, 0, 0),
                    2,
                )

            for point in points:
                cv2.circle(annotated_image, point, 4, (0, 255, 0), -1)

            gesture_name = ''
            if len(result.gestures) > hand_idx and result.gestures[hand_idx]:
                gesture_name = str(result.gestures[hand_idx][0].category_name)

            wrist = hand_landmarks[0]
            text_x = max(int(wrist.x * annotated_image.shape[1]) - 10, 10)
            text_y = max(int(wrist.y * annotated_image.shape[0]) - 10, 30)
            cv2.putText(
                annotated_image,
                gesture_name or 'No gesture',
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return annotated_image

    def process_frame(self):
        if not self.image_received or self.cv_image is None:
            self.get_logger().info('Waiting for image...', throttle_duration_sec=5.0)
            return

        rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        result = self.recognizer.recognize(mp_image)

        gesture_msg = String()
        position_msg = String()
        x_msg = Float64()
        y_msg = Float64()
        annotated_image = self.cv_image.copy()

        if not result.hand_landmarks:
            gesture_msg.data = ''
            position_msg.data = ''
            x_msg.data = 0.0
            y_msg.data = 0.0
        else:
            landmark = result.hand_landmarks[0][9]
            gesture_name = ''
            if result.gestures and result.gestures[0]:
                gesture_name = str(result.gestures[0][0].category_name)

            gesture_msg.data = gesture_name
            position_msg.data = str(landmark)
            x_msg.data = round(float(landmark.x), 4)
            y_msg.data = round(float(landmark.y), 4)
            annotated_image = self.draw_result(self.cv_image, result)

        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        if self.last_image_header is not None:
            annotated_image_msg.header = self.last_image_header

        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)
        self.gesture_pub.publish(gesture_msg)
        self.position_pub.publish(position_msg)
        self.annotated_image_pub.publish(annotated_image_msg)

    def destroy_node(self):
        # Release MediaPipe resources before node shutdown.
        if hasattr(self, 'recognizer') and self.recognizer is not None:
            self.recognizer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognition()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
