import time
import sys
import asyncio
import rclpy
import math
import numpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration  import Duration
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
sys.path.insert(0, "/home/xplanter/planter_ws/src/planter_module")
from gps_navigation_interface.action import Waypoint


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__('waypoint_action_server')

        self.dist = None
        self.r_lat = 0.0
        self.r_lon = 0.0
        self.r_heading = 0.0
        self._action_server = ActionServer(
            self,
            Waypoint,
            'waypoint_action',
            self.execute_callback)

        self.ref_coords1 = self.create_publisher(NavSatFix, '/r2/ref_coordinate1', 5)
        self.ref_coords2 = self.create_publisher(NavSatFix, '/r2/ref_coordinate2', 5)

        self.pub_cmdvel = self.create_publisher(Twist, "/r2/cmd_vel", 5)
        
        self.create_subscription(Float32, '/r2/heading', self.heading_callback, 5)
        self.create_subscription(Float32, '/r2/dist_to_goal_pose', self.dist_to_goal_callback, 5)
        self.create_subscription(NavSatFix,'/r2/gps_agg',self.gps_agg_cb, 5)
        self.e_d_i = 0.0

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal…")

        # ---- 1. Validate goal -----------------------------------------------
        points = goal_handle.request.start_end_points
        if len(points) != 2:
            self.get_logger().error("Goal must contain exactly two NavSatFix points")
            goal_handle.abort()
            return Waypoint.Result()

        nav_point_start, nav_point_end = points
        print(f"Start Point: {nav_point_start}")
        print(f"End Point: {nav_point_end}")
        self.des_lat = nav_point_end.latitude
        self.des_lon = nav_point_end.longitude

        self.ref_coords1.publish(nav_point_start)
        self.ref_coords2.publish(nav_point_end)

        while True:
            dist = self.control_loop()
            if dist < 0.4:
                break;

        # ---- Do the waypoint inside action client
        
        '''
        rate = self.create_rate(1.0)          # 1 Hz feedback

        # ---- 2. Wait until /dist_to_goal_pose starts publishing -------------
        
        while rclpy.ok():
            if self.dist is None:
                rate.sleep()
                continue

            feedback = Waypoint.Feedback()
            feedback.dist = int(self.dist)
            goal_handle.publish_feedback(feedback)

            print(self.dist)

            if self.dist < 0.1:
                break

            rate.sleep()
        '''

        msg_cmd = Twist()
        msg_cmd.linear.x = 0.0
        msg_cmd.angular.z = 0.0
        self.pub_cmdvel.publish(msg_cmd)

        # ---- 4. Finish -------------------------------------------------------
        goal_handle.succeed()

        result = Waypoint.Result()
        result.current_position = NavSatFix()
        result.current_position.latitude  = self.r_lat
        result.current_position.longitude = self.r_lon
        self.e_d_i = 0.0
        return result

    def dist_to_goal_callback(self, msg):
        self.dist = msg.data

    def heading_callback(self, msg):
        self.r_heading = msg.data

    def gps_agg_cb(self, msg):
        self.r_lat = msg.latitude
        self.r_lon = msg.longitude

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x,y)
        brng = 90 -  numpy.degrees(brng) - 88
        if (brng < 0):
            brng = 360 + brng
        return brng

    def get_distance(self, lat1, long1, lat2, long2):
        dLat = (math.radians(lat2) - math.radians(lat1))
        dLon = (math.radians(long2) - math.radians(long1))
        R = 6373.0
        a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R*c*1000
        return distance
    
    def control_loop(self):
        #self.r_heading = msg.data
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(f"rover heading: {self.r_heading}")

        self.des_heading = self.get_bearing(self.r_lat, self.r_lon, self.des_lat, self.des_lon)
        print(f"destination heading: {self.des_heading}")

        self.wp_distance = self.get_distance(self.r_lat, self.r_lon, self.des_lat, self.des_lon)
        print(f"waypoint distance: {self.wp_distance}")
        #print(" ")

        e_h = self.des_heading - self.r_heading
        if e_h > 180:
            e_h -= 360
        if e_h < -180:
            e_h += 360
        print(f"error heading: {e_h}")
        e_d = self.wp_distance
        self.e_d_i = pow(10.0, -5) * e_d + self.e_d_i
        print(f"error distance integral{self.e_d_i}")
        print(f"distance: {e_d}")

        uaz = 0.02*e_h
        if uaz > 0.4:
            uaz = 0.4
        elif uaz < -0.4:
            uaz = -0.4
            
        int_term = self.e_d_i
        
        if int_term > 0.2:
        	int_term = 0.2
        elif int_term < -0.2:
        	int_term = -0.2
        	
        
        ulx = 0.3*e_d + int_term
        if ulx > 0.6:
            ulx = 0.6
        elif ulx < -0.6:
            ulx = -0.6
        msg_cmd = Twist()
        msg_cmd.linear.x = ulx
        msg_cmd.angular.z = uaz
        self.pub_cmdvel.publish(msg_cmd)

        return e_d

def main(args=None):
    rclpy.init(args=args)
    node = WaypointActionServer()

    # Extra threads let dist-updating subscriptions run while execute_callback blocks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
