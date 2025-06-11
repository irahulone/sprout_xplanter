import time
import sys
import configparser
import os
import serial
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer

from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix


import re
sys.path.insert(0, "/home/xplanter/planter_ws/src/planter_module")

#from planter_module.actions.planter import planter
from planter_module.motor_controller import MotorNode
from planter_module.ser import Serial
from planter_action_interface.action import Planter

import time

CONFIG_FILE = "planter_config.cfg"

class PlanterActionServer(Node):

    def __init__(self):
        
        super().__init__('planter_action_server')
        ##### Config init #####
        # Get config dir
        config_file_path = os.getcwd() 
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE)) 
        # Read config file 
        config = configparser.ConfigParser()
        config.read(config_file_path)
        
        # Config Parsing
        self.ROBOT_ID = config['Robot']['id']
        self.MAX_DRILL_DIST = int(config['Drill_Params']['Max_Dist'])
        self.LIN_ACTUATOR_DIST_THRESHOLD = int(config['Drill_Params']['Lin_Act_Dist_Threshold'])
        motor_port = config['Motor_Params']['port']
        linear_port = config['Linear_Params']['port']
        self.motor_ids = [int(x) for x in config['Motor_Params']['ids'].split(', ')]
        motor_baudrate = int(config['Motor_Params']['baudrate'])
        linear_baudrate = int(config['Linear_Params']['baudrate'])
        self.motor_base_speed = int(config['Motor_Params']['speed'])
        self.drill_base_speed = int(config['Drill_Params']['speed'])
        self.linear_base_speed = int(config['Linear_Params']['speed'])
        self.serial = Serial()

        self.dropped_sapling = False

        self.serial.set(config['Arduino_Params']['port'])
        self.roboteq_obj = {}
        
        
        self._default_callback_group = ReentrantCallbackGroup()

        #This is code for the linear actuator
        
        self.roboteq_obj['Linear'] = serial.Serial(
            port=linear_port,
            baudrate=linear_baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)
        self.linear_publisher = self.create_publisher(Int16, 'linear_motor/set_speed', 10)
        self.linear_timer = self.create_timer(1.0, self.publish_linear_speed)

        self.linear_subscriber = self.create_subscription(
                Int16,
                'linear_motor/set_speed',
                self.create_speed_callback(1,'Linear'),
                10)

        msg = Int16()
        msg.data = -400
        self.linear_speed = 0

        # this is code for the drill motor
        self.roboteq_obj['Drill'] = serial.Serial(
            port=linear_port,
            baudrate=linear_baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)
        self.drill_publisher = self.create_publisher(Int16, 'drill_motor/set_speed', 10)
        self.drill_timer = self.create_timer(1.0, self.publish_drill_speed)

        self.drill_subscriber = self.create_subscription(
                Int16,
                'drill_motor/set_speed',
                self.create_speed_callback(2,'Drill'),
                10)
        
        self.drill_speed = 0

        # Conveyor belt motor inits
        # creates roboteq objects and ros2 subscribers based on number of ports defined in config 
        # list inits
        self.speed_subscriber = []
        self.motor_publishers = {}
        self.motor_timers = {}
        self.roboteq_obj['Conveyor'] = serial.Serial(
            port=motor_port,
            baudrate=motor_baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)
        for x in self.motor_ids:
            self.motor_publishers[x] = self.create_publisher(Int16, f'motor_{x}/set_speed', 10)
            self.sequence_subscriber = self.create_subscription(
                Int16,
                f'motor_{x}/set_speed',
                self.create_speed_callback(x,'Conveyor'),
                10)
        #print(self.motor_publishers)
        self.motor_speeds = [0,0,0]
        self.motors_timer = self.create_timer(1.0, self.publish_motor_speeds)
        self._action_server = ActionServer(
            self,
            Planter,
            'planter_action',
            self.execute_callback)
        '''
        self.speed_subscriber.append(self.create_subscription(
            Bool,
            f'run',
            self.execute_callback,
            10))
        '''

    def publish_linear_speed(self):
        msg = Int16()
        msg.data = self.linear_speed
        self.linear_publisher.publish(msg)
        self.get_logger().info(f"Published speed {msg.data} to linear_motor/set_speed")
        return
    
    def publish_drill_speed(self):
        msg = Int16()
        msg.data = self.drill_speed
        self.drill_publisher.publish(msg)
        self.get_logger().info(f"Published speed {msg.data} to drill_motor/set_speed")
        return
    
    def publish_motor_speeds(self):
        for i, motor_id in enumerate(self.motor_ids):
            msg = Int16()
            msg.data = self.motor_speeds[i]
            self.motor_publishers[motor_id].publish(msg)
            self.get_logger().info(f"Published speed {msg.data} to motor_{motor_id}/set_speed")

    
    # Dynamically creates speed callbacks for each motor
    def create_speed_callback(self, motor_id, motor_type):
        def speed_callback(msg):
            command = f"!G {motor_id} {msg.data}_"
            self.roboteq_obj[motor_type].write(str.encode(command))
            self.roboteq_obj[motor_type].flush()
        return speed_callback

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.stage_1()
        self.stage_2(400, 3)   #0 for testting purposes

        goal_handle.succeed()

        result = Planter.Result()

        if self.dropped_sapling:
            result.success = 1
        else:
            result.success = 0

        self.dropped_sapling = False
        return result

    def stage_1(self):
        #time.sleep(2)
        self.linear_speed = -self.linear_base_speed
        start_time = time.time()
        while 1:
            cur_time = time.time()
            if cur_time-start_time < 3:
                break

        self.linear_speed = 0  
        
        data = self.getSensorData()
        print("going down")
        
        start_time = time.time()

        self.drill_speed = -self.drill_base_speed
        self.linear_speed = self.linear_base_speed

        print(int(data["ActuatorDistance"]))
        
        while int(data["ActuatorDistance"]) > self.LIN_ACTUATOR_DIST_THRESHOLD and time.time() - start_time < 15:
            data = self.getSensorData()
            print("Hello world: " + str(data["ActuatorDistance"]))
        
        # Keep the auger spinning the same direction while pulling it back
        print("going up")
        self.linear_speed = -self.linear_base_speed
        start_time = time.time()
        while 1:
            cur_time = time.time()
            if cur_time - start_time >= 20:
                self.linear_speed = 0
                break
        
        self.drill_speed = 0
        
        print("stopping")


    def stage_2(self, speed, motor_id):
        if motor_id > len(self.motor_ids):
            self.get_logger().error(f'Motor id ({motor_id}) out of bounds, number of motors {len(self.motor_ids)}')
            return

        #activate conv grav system
        print("start")
        
        start = time.time()
        current = time.time()

        #goes through all of the converyer belt motors and turns them one by one until at least one of them causes a blacking o occur, which means that the whiile loop will break
        while current - start < 15:
            data =self.getSensorData()

            if current - start <= 5:
                self.motor_speeds = [0, 0, -self.motor_base_speed]
            elif current - start <= 10:
                self.motor_speeds = [0, -self.motor_base_speed, 0]
            else:
                self.motor_speeds = [-self.motor_base_speed, 0, 0]
            
            if int(data["Blocking"]) == 1:
                self.dropped_sapling = True
                break

            current = time.time()
        start = time.time()
        while 1:
            current = time.time()
            if current-start > 2:
                break
        self.motor_speeds = [0, 0, 0]

        return
    
    
    def getSensorData(self):
        data = None
        while data is None:
            recieved_serial = None
            while recieved_serial is None:
                recieved_serial = self.serial.getOutput()
                print(recieved_serial)
            data = self.parse_serial_data(recieved_serial)
            print(data)
        return data

    def parse_serial_data(self, data):
        """
        Parses the serial data string into a dictionary.
        Expected format: "START <auger_dist> <dispenser_dist> <block> <motor_state> END"
        """
        try:
            # Remove newline characters and split the string
            data = str(data)
            print(data)
            parts = data.strip().split()
            print(parts)

            # Validate correct format
            if len(parts) != 6 or parts[0] != "START" or parts[5] != "END":
                raise ValueError("Invalid message format")

            # Extract values and convert to integers
            parsed_data = {
                "ActuatorDistance": int(parts[1]),
                "DispenserDistance": int(parts[2]),
                "Blocking": int(parts[3]),
                "MotorState": int(parts[4])
            }

            return parsed_data
        except Exception as e:
            print(f"Error parsing data: {e}")
            return None

'''
def main(args=None):
    rclpy.init(args=args)
    planter_action_server = PlanterActionServer()
    rclpy.spin(planter_action_server)
'''

def main(args=None):
    rclpy.init(args=args)
    planter_action_server = PlanterActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(planter_action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planter_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
