import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import Call, MoveCartesian, SetInt16, SetInt16ById


class VisualControl(Node):
    def __init__(self):
        super().__init__('visual_control')

        self.declare_parameter('gesture_topic', '/hand/gesture')
        self.declare_parameter('speed', 80.0)
        self.declare_parameter('wait', True)

        self.gesture_topic = str(self.get_parameter('gesture_topic').value)
        self.speed = float(self.get_parameter('speed').value)
        self.wait = bool(self.get_parameter('wait').value)

        self.home = [250.0, 0.0, 90.0, 3.14, 0.0, 0.0]
        self.active_gesture = ''
        self.gripper_state = 'off'
        self.pending_gripper_state = None
        self.gripper_future = None
        self.gripper_command = None
        self.gripper_service_name = ''
        self.gripper_timeout_ns = int(5.0 * 1e9)
        self.gripper_deadline_ns = 0
        self.call_req = Call.Request()

        self.get_logger().info('Initializing robot services...')
        self.init_robot()

        self.get_logger().info('Moving to home and opening gripper...')
        self.send_cartesian(self.home)
        if self.call_gripper_sync('open'):
            self.gripper_state = 'open'

        self.create_subscription(String, self.gesture_topic, self.gesture_callback, 10)
        self.create_timer(0.2, self.control_loop)

    def _wait_service(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')

    def _call_and_wait(self, client, request, service_name: str) -> bool:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(f'Timeout calling {service_name}')
            return False

        if future.result() is None:
            self.get_logger().error(f'Failed calling {service_name}')
            return False

        return True

    def _gripper_service(self, command: str):
        if command == 'open':
            return self.open_gripper_client, '/ufactory/open_lite6_gripper'
        if command == 'closed':
            return self.close_gripper_client, '/ufactory/close_lite6_gripper'
        raise ValueError(f'Unknown gripper command: {command}')

    def call_gripper_sync(self, command: str) -> bool:
        client, service_name = self._gripper_service(command)
        return self._call_and_wait(client, self.call_req, service_name)

    def start_gripper_call(self, command: str) -> bool:
        if self.gripper_future is not None:
            return False

        try:
            client, service_name = self._gripper_service(command)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False

        self.gripper_command = command
        self.gripper_service_name = service_name
        self.gripper_deadline_ns = self.get_clock().now().nanoseconds + self.gripper_timeout_ns
        self.gripper_future = client.call_async(self.call_req)
        self.gripper_future.add_done_callback(
            lambda future, expected=self.gripper_future, cmd=command, srv=service_name:
            self.finish_gripper_call(future, expected, cmd, srv)
        )
        return True

    def finish_gripper_call(self, future, expected_future, command: str, service_name: str):
        if future is not expected_future or self.gripper_future is not expected_future:
            return

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed calling {service_name}: {exc}')
        else:
            if result is None:
                self.get_logger().error(f'Failed calling {service_name}')
            else:
                self.gripper_state = command
                self.get_logger().info(f'Gripper command completed: {command}')

        self.gripper_future = None
        self.gripper_command = None
        self.gripper_service_name = ''

    def init_robot(self):
        self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        self.set_position_client = self.create_client(MoveCartesian, '/ufactory/set_position')
        self.close_gripper_client = self.create_client(Call, '/ufactory/close_lite6_gripper')
        self.open_gripper_client = self.create_client(Call, '/ufactory/open_lite6_gripper')

        self._wait_service(self.motion_enable_client, '/ufactory/motion_enable')
        self._wait_service(self.set_mode_client, '/ufactory/set_mode')
        self._wait_service(self.set_state_client, '/ufactory/set_state')
        self._wait_service(self.set_position_client, '/ufactory/set_position')
        self._wait_service(self.close_gripper_client, '/ufactory/close_lite6_gripper')
        self._wait_service(self.open_gripper_client, '/ufactory/open_lite6_gripper')

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
        self.active_gesture = self.normalize_gesture(msg.data)

    def send_cartesian(self, pose):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        req.pose = pose
        self._call_and_wait(self.set_position_client, req, '/ufactory/set_position')

    def control_loop(self):
        desired_state = {
            'open_palm': 'open',
            'closed_fist': 'closed',
        }.get(self.active_gesture)

        if (
            desired_state is not None
            and desired_state != self.gripper_state
            and desired_state != self.pending_gripper_state
            and desired_state != self.gripper_command
        ):
            self.get_logger().info(f'Queueing gripper {desired_state}')
            self.pending_gripper_state = desired_state

        if (
            self.gripper_future is not None
            and self.get_clock().now().nanoseconds > self.gripper_deadline_ns
        ):
            self.get_logger().error(f'Timeout calling {self.gripper_service_name}')
            self.gripper_future.cancel()
            self.gripper_future = None
            self.gripper_command = None
            self.gripper_service_name = ''

        if self.gripper_future is not None or self.pending_gripper_state is None:
            return

        self.get_logger().info(f'Executing gripper {self.pending_gripper_state}')
        self.start_gripper_call(self.pending_gripper_state)
        self.pending_gripper_state = None


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
