import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
sys.path.insert(0, "/home/xplanter/planter_ws/src/planter_module")
from gps_navigation_interface.action import Waypoint

class WaypointActionClient(Node):

    def __init__(self):
        super().__init__('waypoint_action_client')
        self._action_client = ActionClient(self, Waypoint, 'waypoint_action')
        self.goal_done = False

    def send_goal(self, order):
        goal_msg = Waypoint.Goal()
        goal_msg.start_end_points = order

        self._action_client.wait_for_server()

        self.goal_done =False
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

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
        self.get_logger().info(f"Result for goal: {result.current_position}")
        self.goal_done = True


def main(args=None):
    rclpy.init(args=args)

    action_client = WaypointActionClient()

    goals = [[
                NavSatFix(latitude=37.260269949999994, longitude=-121.8432234),  # Example data
                NavSatFix(latitude=37.260269949999994, longitude=-121.8432234)  # Example data
            ],
            [
                NavSatFix(latitude=37.26031365, longitude=-121.84322435),  
                NavSatFix(latitude=37.26031365, longitude=-121.84322435)  
            ],
            [
                NavSatFix(latitude=37.2603682, longitude=-121.8432271),  
                NavSatFix(latitude=37.2603682, longitude=-121.8432271) 
            ],
            [
                NavSatFix(latitude=37.2603646, longitude=-121.84322434999999),  
                NavSatFix(latitude=37.2603646, longitude=-121.84322434999999)  
            ],
            [
                NavSatFix(latitude=37.2605576, longitude=-121.84322505),  
                NavSatFix(latitude=37.2605576, longitude=-121.84322505)  
            ]
    ]

    for i, goal_data in enumerate(goals):
        action_client.get_logger().info(f"Sending goal {i + 1}/{len(goals)}...")
        action_client.send_goal(goal_data)
        while not action_client.goal_done:
            rclpy.spin_once(action_client, timeout_sec=0.1)  # Allows processing of callbacks


    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
