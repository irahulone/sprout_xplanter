import rclpy
import sys
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

sys.path.insert(0, "/home/xplanter/planter_ws/src/planter_module")
from planter_action_interface.action import Planter


class PlanterActionClient(Node):

    def __init__(self):
        super().__init__('planter_action_client')
        self._action_client = ActionClient(self, Planter, 'planter_action')
        self.planting_success = False
        self.goal_done = False  # Flag to indicate goal completion

    def send_goal(self, order):
        goal_msg = Planter.Goal()
        goal_msg.num_waypoint = order

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available. Sending goal...")

        self.goal_done = False  # Reset flag
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")

        self.planting_success = result.success
        self.goal_done = True  # Indicate that goal processing is finished


def main(args=None):
    rclpy.init(args=args)
    action_client = PlanterActionClient()

    for i in range(5):
        print(f"Sending goal {i+1}/5...")
        action_client.send_goal(1)

        # Wait for goal completion
        while not action_client.goal_done:
            rclpy.spin_once(action_client, timeout_sec=0.1)  # Allows processing of callbacks

        print(f'Success: {action_client.planting_success}')
        print("Goal completed.")
        input("Press Enter to send the next goal...")

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
