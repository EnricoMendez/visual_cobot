import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import Call, MoveCartesian, SetInt16, SetInt16ById


class VisualControl(Node):
    def __init__(self):
        super().__init__('visual_control')

        # Parameters
        self.declare_parameter('gesture_topic', '/hand/gesture')
        self.declare_parameter('speed', 80.0)
        self.declare_parameter('wait', True)

        self.gesture_topic = str(self.get_parameter('gesture_topic').value)
        self.speed = float(self.get_parameter('speed').value)
        self.wait = bool(self.get_parameter('wait').value)

        # Pose state [x, y, z, roll, pitch, yaw]
        self.home = [250.0, 0.0, 90.0, 3.14, 0.0, 0.0]
        self.pose = self.home.copy()

        # Gesture state
        self.active_gesture = ''
        self.last_executed_gesture = ''

        # Reusable empty request for gripper services
        self.call_req = Call.Request()

        self.get_logger().info('Initializing robot services...')
        self.init_robot()

        self.get_logger().info('Moving to home and opening gripper...')
        self.send_cartesian(self.home)
        self.open_gripper()

        # Subscriber
        self.create_subscription(
            String,
            self.gesture_topic,
            self.gesture_callback,
            10
        )

        # Timer
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _wait_service(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')

    def _call_and_wait(self, client, request, service_name: str) -> bool:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Failed calling {service_name}')
            return False

        return True

    def init_robot(self):
        self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        self.set_position_client = self.create_client(MoveCartesian, '/ufactory/set_position')
        self.close_gripper_client = self.create_client(Call, '/ufactory/close_lite6_gripper')
        self.open_gripper_client = self.create_client(Call, '/ufactory/open_lite6_gripper')
        self.stop_gripper_client = self.create_client(Call, '/ufactory/stop_lite6_gripper')

        self._wait_service(self.motion_enable_client, '/ufactory/motion_enable')
        self._wait_service(self.set_mode_client, '/ufactory/set_mode')
        self._wait_service(self.set_state_client, '/ufactory/set_state')
        self._wait_service(self.set_position_client, '/ufactory/set_position')
        self._wait_service(self.close_gripper_client, '/ufactory/close_lite6_gripper')
        self._wait_service(self.open_gripper_client, '/ufactory/open_lite6_gripper')
        self._wait_service(self.stop_gripper_client, '/ufactory/stop_lite6_gripper')

        motion_req = SetInt16ById.Request()
        motion_req.id = 8
        motion_req.data = 1
        self._call_and_wait(self.motion_enable_client, motion_req, '/ufactory/motion_enable')

        mode_req = SetInt16.Request()
        mode_req.data = 0
        self._call_and_wait(self.set_mode_client, mode_req, '/ufactory/set_mode')

        state_req = SetInt16.Request()
        state_req.data = 0
        self._call_and_wait(self.set_state_client, state_req, '/ufactory/set_state')

    def normalize_gesture(self, raw: str) -> str:
        text = raw.strip().lower().replace('-', ' ').replace('_', ' ')

        aliases = {
            'closed fist': 'closed_fist',
            'fist': 'closed_fist',
            'puño cerrado': 'closed_fist',
            'puno cerrado': 'closed_fist',

            'open palm': 'open_palm',
            'open hand': 'open_palm',
            'mano abierta': 'open_palm',
            'mano apierta': 'open_palm',
        }

        return aliases.get(text, text.replace(' ', '_'))

    def gesture_callback(self, msg: String):
        gesture = self.normalize_gesture(msg.data)
        self.active_gesture = gesture
        self.get_logger().info(f'Received gesture: "{msg.data}" -> "{gesture}"')

    def execute_gesture(self):
        gesture = self.active_gesture

        if not gesture:
            return 'idle'

        # Evita mandar el mismo comando repetidamente
        if gesture == self.last_executed_gesture:
            return f'noop: {gesture} already executed'

        if gesture == 'open_palm':
            self.get_logger().info('Opening gripper...')
            self.open_gripper()
            self.last_executed_gesture = gesture
            return 'opened gripper'

        if gesture == 'closed_fist':
            self.get_logger().info('Closing gripper...')
            self.close_gripper()
            self.last_executed_gesture = gesture
            return 'closed gripper'

        # Si llega un gesto no reconocido, no hacemos nada
        self.last_executed_gesture = ''
        return f'unknown gesture: {gesture}'

    def timer_callback(self):
        result = self.execute_gesture()
        if not result.startswith('noop') and result != 'idle':
            self.get_logger().info(result)

    def send_cartesian(self, pose=None):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        req.pose = self.pose if pose is None else pose

        ok = self._call_and_wait(self.set_position_client, req, '/ufactory/set_position')
        if ok and pose is not None:
            self.pose = pose.copy()

    def close_gripper(self):
        self._call_and_wait(
            self.close_gripper_client,
            self.call_req,
            '/ufactory/close_lite6_gripper'
        )

    def open_gripper(self):
        self._call_and_wait(
            self.open_gripper_client,
            self.call_req,
            '/ufactory/open_lite6_gripper'
        )

        # Opcional: detener después de un poco de tiempo
        time.sleep(1.0)
        self.stop_gripper()

    def stop_gripper(self):
        self._call_and_wait(
            self.stop_gripper_client,
            self.call_req,
            '/ufactory/stop_lite6_gripper'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisualControl()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()