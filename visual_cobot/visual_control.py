import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import Call, MoveCartesian, SetInt16, SetInt16ById


class VisualControl(Node):
    def __init__(self):
        super().__init__('visual_control')

        # Parameters let us change the topic and motion settings from a launch file.
        self.declare_parameter('gesture_topic', '/hand/gesture')
        self.declare_parameter('speed', 80.0)

        self.gesture_topic = str(self.get_parameter('gesture_topic').value)
        self.speed = float(self.get_parameter('speed').value)
        self.wait = True

        # Home pose used when the node starts.
        self.home = [250.0, 0.0, 90.0, 3.14, 0.0, 0.0]

        # Latest gesture from the camera and the previous one.
        # Comparing both helps us react only when the gesture changes.
        self.active_gesture = ''
        self.last_gesture = ''

        # Gripper state tracking.
        # We also keep the current async service call so the timer never blocks.
        self.gripper_state = 'unknown'
        self.gripper_future = None
        self.gripper_target = None
        self.gripper_timeout_ns = int(5.0 * 1e9)
        self.gripper_deadline_ns = 0
        self.call_req = Call.Request()

        self.get_logger().info('Initializing robot services...')
        self.initialize_robot()

        self.get_logger().info('Moving to home and opening gripper...')
        self.send_cartesian(self.home)
        self.call_gripper('open')

        # Subscriber: receives the gesture detected by the vision node.
        self.create_subscription(String, self.gesture_topic, self.gesture_callback, 10)

        # Timer: this is the small control loop of the node.
        self.create_timer(0.2, self.check_gesture)

    def _wait_service(self, client, name: str):
        # Wait until a ROS service is available before using it.
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')

    def _call_and_wait(self, client, request, service_name: str) -> bool:
        # Generic helper for services where we do want to wait for the response.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(f'Timeout calling {service_name}')
            return False

        if future.result() is None:
            self.get_logger().error(f'Failed calling {service_name}')
            return False

        return True

    def call_gripper(self, target_state: str):
        # Do nothing if a gripper command is already running
        # or if the gripper is already in the requested state.
        if self.gripper_future is not None or target_state == self.gripper_state:
            return

        # Each logical state maps to one ROS service.
        service_map = {
            'open': (self.open_gripper_client, '/ufactory/open_lite6_gripper'),
            'closed': (self.close_gripper_client, '/ufactory/close_lite6_gripper'),
        }

        if target_state not in service_map:
            self.get_logger().error(f'Unknown gripper state: {target_state}')
            return

        client, service_name = service_map[target_state]
        self.gripper_target = target_state
        self.gripper_deadline_ns = self.get_clock().now().nanoseconds + self.gripper_timeout_ns

        # Start the service asynchronously so the timer can keep running.
        self.gripper_future = client.call_async(self.call_req)
        self.get_logger().info(f'Starting gripper command: {target_state}')

    def update_gripper_call(self):
        # If no command is running, there is nothing to update.
        if self.gripper_future is None:
            return

        # While the service is still running, only check for timeout.
        if not self.gripper_future.done():
            if self.get_clock().now().nanoseconds > self.gripper_deadline_ns:
                self.get_logger().error('Gripper command timed out')
                self.gripper_future.cancel()
                self.gripper_future = None
                self.gripper_target = None
            return

        # Once the service finishes, update the saved gripper state.
        try:
            result = self.gripper_future.result()
        except Exception as exc:
            self.get_logger().error(f'Gripper command failed: {exc}')
        else:
            if result is None:
                self.get_logger().error('Gripper command failed')
            else:
                self.gripper_state = self.gripper_target
                self.get_logger().info(f'Gripper is now {self.gripper_state}')

        self.gripper_future = None
        self.gripper_target = None

    def initialize_robot(self):
        # Create all ROS clients used by this node.
        self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        self.set_position_client = self.create_client(MoveCartesian, '/ufactory/set_position')
        self.close_gripper_client = self.create_client(Call, '/ufactory/close_lite6_gripper')
        self.open_gripper_client = self.create_client(Call, '/ufactory/open_lite6_gripper')

        # Make sure every service exists before continuing.
        self._wait_service(self.motion_enable_client, '/ufactory/motion_enable')
        self._wait_service(self.set_mode_client, '/ufactory/set_mode')
        self._wait_service(self.set_state_client, '/ufactory/set_state')
        self._wait_service(self.set_position_client, '/ufactory/set_position')
        self._wait_service(self.close_gripper_client, '/ufactory/close_lite6_gripper')
        self._wait_service(self.open_gripper_client, '/ufactory/open_lite6_gripper')

        # Basic robot initialization sequence.
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
        # Convert different spellings of the same gesture to one common name.
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
        # Save the last gesture reported by the vision system.
        self.active_gesture = self.normalize_gesture(msg.data)

    def send_cartesian(self, pose):
        # Move the robot TCP to a Cartesian pose.
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        req.pose = pose
        self._call_and_wait(self.set_position_client, req, '/ufactory/set_position')

    def check_gesture(self):
        # First, update the status of any running gripper command.
        self.update_gripper_call()

        # If the gripper is still busy, wait for the next timer cycle.
        if self.gripper_future is not None:
            return

        # Ignore empty gestures and repeated gestures.
        if not self.active_gesture or self.active_gesture == self.last_gesture:
            return

        # Translate each recognized gesture into a gripper action.
        gesture_to_state = {
            'open_palm': 'open',
            'closed_fist': 'closed',
        }
        target_state = gesture_to_state.get(self.active_gesture)

        if target_state is not None:
            self.call_gripper(target_state)

        # Save the gesture so we only react again when it changes.
        self.last_gesture = self.active_gesture


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
