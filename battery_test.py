import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, 'battery_status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        self.battery_level = 1.0

    def publish_battery_status(self):
        msg = BatteryState()
        # Simulate battery drain
        self.battery_level -= random.uniform(0.01, 0.05)
        self.battery_level = max(self.battery_level, 0.0)

        msg.percentage = self.battery_level
        msg.voltage = 12.5 * self.battery_level
        msg.current = -1.0 * (1.0 - self.battery_level)
        msg.charge = self.battery_level * 100.0
        msg.capacity = 100.0
        msg.design_capacity = 100.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery: {msg.percentage * 100:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)  # <--- This keeps it running!
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


