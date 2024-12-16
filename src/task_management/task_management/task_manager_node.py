import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.publisher = self.create_publisher(String, 'task_topic', 10)
        self.timer = self.create_timer(2.0, self.publish_tasks)
        self.orders = self.load_orders()
        self.task_index = 0

    def load_orders(self):
        data = [
            {"client": "client1", "Shelf_1": "D", "Quantity_1": 19},
            {"client": "client2", "Shelf_1": "B", "Quantity_1": 16},
            {"client": "client3", "Shelf_1": "A", "Quantity_1": 17},
        ]
        return data

    def publish_tasks(self):
        if self.task_index < len(self.orders):
            task = self.orders[self.task_index]
            msg = String()
            msg.data = f"Client: {task['client']}, Shelf: {task['Shelf_1']}, Quantity: {task['Quantity_1']}"
            self.publisher.publish(msg)
            self.get_logger().info(f"Published Task: {msg.data}")
            self.task_index += 1
        else:
            self.get_logger().info("All tasks have been published.")

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

