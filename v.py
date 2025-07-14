import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rich.progress import Progress, BarColumn, TextColumn
from rich.live import Live
from time import sleep

class BatteryVisualizer(Node):
    def __init__(self):
        super().__init__('battery_visualizer')
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.listener_callback,
            10)
        self.battery_percentage = 1.0
        self.progress = Progress(
            TextColumn("[bold blue]Battery:[/bold blue]"),
            BarColumn(bar_width=60),
            TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        )
        self.task = self.progress.add_task("battery", total=100)

    def listener_callback(self, msg: BatteryState):
        self.battery_percentage = msg.percentage * 100
        self.progress.update(self.task, completed=self.battery_percentage)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryVisualizer()

    with Live(node.progress, refresh_per_second=10):
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                sleep(0.1)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()