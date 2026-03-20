import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class VisualControlSim(Node):
    def __init__(self):
        super().__init__('visual_control_sim')

        self.declare_parameter('gesture_topic', '/hand/gesture')
        self.declare_parameter(
            'trajectory_action',
            '/lite6_traj_controller/follow_joint_trajectory',
        )
        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        )
        self.declare_parameter('home_joints', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('trajectory_duration', 3.0)
        self.declare_parameter('use_sim_time', True)

        self.gesture_topic = str(self.get_parameter('gesture_topic').value)
        self.trajectory_action = str(self.get_parameter('trajectory_action').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.home_joints = [float(value) for value in self.get_parameter('home_joints').value]
        self.trajectory_duration = float(self.get_parameter('trajectory_duration').value)

        self.active_gesture = ''
        self.last_gesture = ''
        self.gripper_state = 'open'
        self.gripper_warning_emitted = False

        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action,
        )

        self.get_logger().info(
            f'Waiting for action server {self.trajectory_action}...'
        )
        if not self.follow_joint_trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f'Action server {self.trajectory_action} not available.'
            )

        self.get_logger().info('Moving simulated robot to home...')
        self.send_joint_trajectory(self.home_joints)

        self.create_subscription(String, self.gesture_topic, self.gesture_callback, 10)
        self.create_timer(0.2, self.control_loop)

    def _seconds_to_duration_msg(self, seconds: float):
        whole_seconds = max(0, int(seconds))
        nanoseconds = max(0, int((seconds - whole_seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return rclpy.duration.Duration(
            seconds=whole_seconds,
            nanoseconds=nanoseconds,
        ).to_msg()

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

    def send_joint_trajectory(self, positions):
        if len(positions) != len(self.joint_names):
            self.get_logger().error(
                'home_joints and joint_names must have the same length.'
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = self._seconds_to_duration_msg(self.trajectory_duration)
        goal.trajectory.points = [point]

        goal_future = self.follow_joint_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        if not goal_future.done() or goal_future.result() is None:
            self.get_logger().error(
                f'Failed sending goal to {self.trajectory_action}'
            )
            return False

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal rejected by {self.trajectory_action}'
            )
            return False

        result_future = goal_handle.get_result_async()
        wait_timeout = max(5.0, self.trajectory_duration + 2.0)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=wait_timeout)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(
                f'Timeout waiting for result from {self.trajectory_action}'
            )
            return False

        result = result_future.result().result
        if result.error_code != 0:
            self.get_logger().error(
                f'Trajectory execution failed with code {result.error_code}: '
                f'{result.error_string}'
            )
            return False

        return True

    def warn_gripper_unsupported(self):
        if self.gripper_warning_emitted:
            return
        self.gripper_warning_emitted = True
        self.get_logger().warning(
            'Lite6 Gazebo in xarm_ros2 does not expose the gripper controller; '
            'open/close gestures are handled as a no-op.'
        )

    def control_loop(self):
        if not self.active_gesture or self.active_gesture == self.last_gesture:
            return

        if self.active_gesture in ('open_palm', 'closed_fist'):
            desired_state = 'open' if self.active_gesture == 'open_palm' else 'closed'
            if desired_state != self.gripper_state:
                self.warn_gripper_unsupported()
                self.gripper_state = desired_state
                self.get_logger().info(
                    f'Simulated gesture received: {self.active_gesture} -> {desired_state}'
                )

        self.last_gesture = self.active_gesture


def main(args=None):
    rclpy.init(args=args)
    node = VisualControlSim()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
