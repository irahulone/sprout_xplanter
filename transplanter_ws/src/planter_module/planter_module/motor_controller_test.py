import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Int16

class MotorSpeedPublisher(Node):
    def __init__(self, motor_id):
        super().__init__("motor_speed_publisher")
        self.motor_id = motor_id
        self.publisher = self.create_publisher(Int16, f"motor_{motor_id}/set_speed", 10)
        self.timer = self.create_timer(1.0, self.publish_speed)  # Publish every second
        self.speed = 200  # Default speed

    def publish_speed(self):
        msg = Int16()
        msg.data = self.speed
        self.publisher.publish(msg)
        self.get_logger().info(f"Published speed {self.speed} to motor_{self.motor_id}/set_speed")

    def set_speed(self, speed):
        self.speed = speed

def main(args=None):
    rclpy.init(args=args)
    print(len(sys.argv))
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1  # Get motor ID from args
    node = MotorSpeedPublisher(motor_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
