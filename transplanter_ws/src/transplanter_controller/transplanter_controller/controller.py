import rclpy
import math
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix

from planter_module.planter_module_client import PlanterActionClient
from gps_navigation_module.waypoint_action_client import WaypointActionClient


def generate_waypoints(corner1, corner2, horizontal_spacing, vertical_spacing):
    """
    Generate a grid of waypoints given two diagonally opposing corners and spacing in meters.
    
    Parameters:
        corner1 (tuple): (lat, lon) for the first corner.
        corner2 (tuple): (lat, lon) for the opposite corner.
        horizontal_spacing (float): spacing between waypoints horizontally (in meters).
        vertical_spacing (float): spacing between waypoints vertically (in meters).
    
    Returns:
        List[tuple]: A list of waypoints as (lat, lon) tuples.
    """
    # Unpack corners
    lat1, lon1 = corner1
    lat2, lon2 = corner2
    if lat1 is None or lat2 is None:
        return None
    elif lon1 is None or lon2 is None:
        return None
    # Determine min and max in latitude and longitude
    lat_min = min(lat1, lat2)
    lat_max = max(lat1, lat2)
    lon_min = min(lon1, lon2)
    lon_max = max(lon1, lon2)

    # Conversion factors (approximate)
    meters_per_deg_lat = 111320  # ~111.32 km per degree latitude
    avg_lat = (lat_min + lat_max) / 2.0
    meters_per_deg_lon = 111320 * math.cos(math.radians(avg_lat))

    # Calculate total distances in meters in each direction
    total_vertical_m = (lat_max - lat_min) * meters_per_deg_lat
    total_horizontal_m = (lon_max - lon_min) * meters_per_deg_lon

    # Compute the number of waypoints along each direction.
    # Add one extra point to include both boundaries.
    num_rows = int(total_vertical_m / vertical_spacing) + 1
    num_cols = int(total_horizontal_m / horizontal_spacing) + 1

    # Calculate the increments in degrees based on the spacing in meters.
    delta_lat = vertical_spacing / meters_per_deg_lat
    delta_lon = horizontal_spacing / meters_per_deg_lon

    waypoints = []
    # We start at the top (lat_max) and move down, and from lon_min it will move in a snakelike fashion.
    for row in range(num_rows):
        current_lat = lat_max - row * delta_lat  # moving southward decreases latitude
        for col in range(num_cols):
            current_lon = lon_min + col * delta_lon
            if row%2 != 0:
                current_lon = lon_max - col * delta_lon
            waypoints.append([current_lat, current_lon])

    nav_sat_fix_waypoints = []

    for i in range(1, len(waypoints)):
        nav_sat_fix_waypoints.append([NavSatFix(latitude=waypoints[i-1][0], longitude=waypoints[i-1][1]), 
                                      NavSatFix(latitude=waypoints[i][0], longitude=waypoints[i][1])])

    return nav_sat_fix_waypoints



def main(args=None):
    rclpy.init(args=args)
    plant_action = PlanterActionClient()
    path_action = WaypointActionClient()

    '''
    goals = [[
                NavSatFix(latitude=37.2602735, longitude=-121.84319704999999),  
                NavSatFix(latitude=37.2602735, longitude=-121.84319704999999)
            ],
            [
                NavSatFix(latitude=37.26027795, longitude=-121.84319685),
                NavSatFix(latitude=37.26027795, longitude=-121.84319685)
            ],
            [
                NavSatFix(latitude=37.26028365, longitude=-121.84319715),
                NavSatFix(latitude=37.26028365, longitude=-121.84319715)
            ],
            [
                NavSatFix(latitude=37.260289349999994, longitude= -121.84319715),
                NavSatFix(latitude=37.260289349999994, longitude= -121.84319715)
            ],
            [
                NavSatFix(latitude=37.260294599999995, longitude= -121.84319679999999),
                NavSatFix(latitude=37.260294599999995, longitude= -121.84319679999999)
            ]
    ]
    '''
    bottom_left_corner = (37.26013215,-121.84360124999999)
    top_right_corner = (37.259952, -121.8437572)
    horizontal_spacing = 5
    vertical_spacing = 5
    #goals = generate_waypoints(top_right_corner, bottom_left_corner, horizontal_spacing, vertical_spacing)
    lat_space = 5.35*(10**-6)
    lon_space = -2.5*(10**-7)
    start_point = (37.26023775, -121.8432481)
    goals = []

    for i in range(10):
        point = NavSatFix(latitude = start_point[0]+lat_space * i, longitude = start_point[1] + lon_space * i)
        print(point)
        goals.append([point, point])

        

    if goals is None:
        exit()

    for i, goal_data in enumerate(goals):
        print(goal_data)  
        path_action.send_goal(goal_data)
        while not path_action.goal_done:
            rclpy.spin_once(path_action, timeout_sec=0.1)
        
        print(f"Sending goal {i+1}/5...")
        plant_action.send_goal(1)

        # Wait for goal completion
        while not plant_action.goal_done:
            rclpy.spin_once(plant_action, timeout_sec=0.1)  # Allows processing of callbacks

        print(f'Success: {plant_action.planting_success}')
        print("Goal completed.")

    plant_action.destroy_node()
    path_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
