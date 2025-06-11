#!/usr/bin/env python3
#
# waypoint_heading_controller.py – cleaned up for ROS 2 Humble

import math
import time  # ← still needed for the 1 s sleep when goal reached
from typing import List

import numpy as np

import rclpy
from rclpy.node           import Node
from geometry_msgs.msg    import Twist
from std_msgs.msg         import Float32, Int16, Bool
from sensor_msgs.msg      import NavSatFix


class WaypointController(Node):

    # ----------------- tunables / constants --------------------------
    HEADING_ERROR_INT_GAIN_CUTOFF = 30        # deg
    DIST_TOLERANCE_CLOSE_PATH     = 1.5       # m
    DIST_TOLERANCE_FAR_PATH       = 0.5       # m
    DIST_STOP_REACHED             = 0.10      # m
    KP_HEADING                    = 0.008
    KI_HEADING                    = 0.0003
    KP_LINEAR                     = 0.3
    MAX_LIN_SPEED                 = 0.22      # m/s
    MAX_ANG_SPEED                 = 0.40      # rad/s
    FEEDBACK_PERIOD               = 0.1       # s (10 Hz)

    # -----------------------------------------------------------------

    def __init__(self) -> None:
        super().__init__("waypoint_heading_controller")

        # State ----------------------------------------------------------------
        self.rover_heading       : float = 0.0
        self.heading_error_int   : float = 0.0
        self.rover_lat           : float = 0.0
        self.rover_lon           : float = 0.0

        # next path (2 way-points published by the waypoint server)
        self.ref_lat1            : float = 0.0
        self.ref_lon1            : float = 0.0
        self.ref_lat2            : float = 0.0
        self.ref_lon2            : float = 0.0

        self.path_id             : int   = 0
        self.goal_reached_once   : bool  = False   # “f1” in original code
        self.cmd                 : Twist = Twist()

        # Publishers ------------------------------------------------------------
        self.pub_cmd_vel         = self.create_publisher(Twist,     "/r2/cmd_vel",            5)
        self.pub_ref_heading     = self.create_publisher(Float32,   "/r2/ref_heading",        5)
        self.pub_path_bearing    = self.create_publisher(Float32,   "/r2/path_bearing",       5)
        self.pub_xte             = self.create_publisher(Float32,   "/r2/xte",                5)
        self.pub_exe_status      = self.create_publisher(Bool,      "/r2/exe_status",         5)
        self.pub_dist_to_goal    = self.create_publisher(Float32,   "/r2/dist_to_goal_pose",  5)
        self.pub_rover_pos       = self.create_publisher(NavSatFix, "/r2/rover_gps",          5)

        # Subscribers -----------------------------------------------------------
        self.create_subscription(Float32,  "/r2/heading",        self.cb_rover_heading,  5)
        self.create_subscription(Int16,    "/r2/path_id",        self.cb_path_id,        5)
        self.create_subscription(NavSatFix,"/r2/gps_agg",       self.cb_gps,           10)
        self.create_subscription(NavSatFix,"/r2/ref_coordinate1", self.cb_ref_coord1,   5)
        self.create_subscription(NavSatFix,"/r2/ref_coordinate2", self.cb_ref_coord2,   5)

        # Main control loop -----------------------------------------------------
        self.create_timer(self.FEEDBACK_PERIOD, self.control_loop)

    # --------------- callbacks that just copy incoming data -------------------
    def cb_rover_heading(self, msg: Float32) -> None:
        self.rover_heading = msg.data

    def cb_path_id(self, msg: Int16) -> None:
        self.path_id = msg.data

    def cb_gps(self, msg: NavSatFix) -> None:
        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude

    def cb_ref_coord1(self, msg: NavSatFix) -> None:
        self.ref_lat1 = msg.latitude
        self.ref_lon1 = msg.longitude

    def cb_ref_coord2(self, msg: NavSatFix) -> None:
        self.ref_lat2 = msg.latitude
        self.ref_lon2 = msg.longitude

    # ----------------- helper math -------------------------------------------
    @staticmethod
    def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle bearing from point 1 to point 2, degrees [0–360)."""
        dlon  = math.radians(lon2 - lon1)
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)

        x = math.cos(lat2r) * math.sin(dlon)
        y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
        brng = math.degrees(math.atan2(x, y))
        return (brng + 360.0) % 360.0

    @staticmethod
    def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance in metres."""
        R = 6_373_000.0  # Earth radius, m
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1r) * math.cos(lat2r) * math.sin(dlon / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def saturate(val: float, upper: float, lower: float) -> float:
        return max(min(val, upper), lower)

    # ----------------- main control loop -------------------------------------
    def control_loop(self) -> None:
        # 1. Path geometry ------------------------------------------------------
        path_bearing = self.bearing_deg(
            self.ref_lat1, self.ref_lon1,
            self.ref_lat2, self.ref_lon2
        )
        path_bearing = (360.0 - path_bearing) % 360.0

        dist_between_refs = self.haversine_m(
            self.ref_lat1, self.ref_lon1,
            self.ref_lat2, self.ref_lon2
        )
        dist_to_goal = self.haversine_m(
            self.rover_lat, self.rover_lon,
            self.ref_lat2, self.ref_lon2
        )

        # Cross-track error (straight-line distance to the reference line) -----
        dx = self.ref_lon2 - self.ref_lon1
        dy = self.ref_lat2 - self.ref_lat1
        if abs(dx) < 1e-12:                       # avoid /0 when path is N-S
            slope = float("inf")
        else:
            slope = dy / dx
        c = self.ref_lat1 - slope * self.ref_lon1
        denom = math.sqrt(slope ** 2 + 1.0)
        d = (slope * self.rover_lon - self.rover_lat + c) / denom
        xte = d * 100_000.0                       # scale to metres-ish

        # 2. Desired heading ----------------------------------------------------
        ref_heading = (path_bearing - 10.0 * xte) % 360.0

        # 3. Execution-status flag / stop when close ----------------------------
        tol = (self.DIST_TOLERANCE_CLOSE_PATH if dist_between_refs < 14.0
               else self.DIST_TOLERANCE_FAR_PATH)
        run_status = Bool(data=False)

        if dist_to_goal < tol and not self.goal_reached_once:
            # first time inside tolerance – stop & debounce 1 s
            self.goal_reached_once = True
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            run_status.data = True
            self.pub_cmd_vel.publish(self.cmd)
            self.pub_exe_status.publish(run_status)
            time.sleep(1.0)
        else:
            self.goal_reached_once = False

        # 4. Heading controller -------------------------------------------------
        heading_error = (ref_heading - self.rover_heading + 540.0) % 360.0 - 180.0

        # PI controller with integral anti-wind-up
        if abs(heading_error) < self.HEADING_ERROR_INT_GAIN_CUTOFF:
            self.heading_error_int += heading_error
        else:
            self.heading_error_int = 0.0

        uz_raw = (self.KP_HEADING * heading_error +
                  self.KI_HEADING * self.heading_error_int)
        uz = self.saturate(uz_raw, self.MAX_ANG_SPEED, -self.MAX_ANG_SPEED)

        # 5. Linear speed profile ----------------------------------------------
        k_lx = 1.0 - self.saturate(0.02 * abs(heading_error), 1.0, 0.0)
        ux_raw = self.KP_LINEAR * dist_to_goal
        ux = k_lx * self.saturate(ux_raw, self.MAX_LIN_SPEED, -self.MAX_LIN_SPEED)

        # 6. Publish all --------------------------------------------------------
        self.cmd.linear.x  = ux
        self.cmd.angular.z = uz
        self.pub_cmd_vel.publish(self.cmd)

        self.pub_ref_heading.publish(Float32(data=float(ref_heading)))
        self.pub_path_bearing.publish(Float32(data=float(path_bearing)))
        self.pub_xte.publish(Float32(data=float(xte)))
        self.pub_exe_status.publish(run_status)
        self.pub_dist_to_goal.publish(Float32(data=float(dist_to_goal)))

        rover_pose = NavSatFix(latitude=self.rover_lat, longitude=self.rover_lon)
        self.pub_rover_pos.publish(rover_pose)

        # Debug prints (remove for production) ---------------------------------
        self.get_logger().info(
            f"u_lin={ux:+.2f} m/s, u_ang={uz:+.2f} rad/s, "
            f"hdg_err={heading_error:+.1f}°, dist={dist_to_goal:.1f} m, xte={xte:+.1f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down waypoint controller…")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
