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
            {"client": "client1", "Shelf_1": "D", "Quantity_1": 19, "Shelf_2": "A", "Quantity_2": 4, "Shelf_3": "B", "Quantity_3": 23, "Shelf_4": "C", "Quantity_4": 11},
	    {"client": "client2", "Shelf_1": "B", "Quantity_1": 16, "Shelf_2": "C", "Quantity_2": 8, "Shelf_3": "A", "Quantity_3": 1, "Shelf_4": "D", "Quantity_4": 20},
	    {"client": "client3", "Shelf_1": "A", "Quantity_1": 17, "Shelf_2": "B", "Quantity_2": 7, "Shelf_3": "C", "Quantity_3": 5, "Shelf_4": "D", "Quantity_4": 14},
	    {"client": "client4", "Shelf_1": "A", "Quantity_1": 20, "Shelf_2": "B", "Quantity_2": 8, "Shelf_3": "C", "Quantity_3": 19, "Shelf_4": "D", "Quantity_4": 11},
	    {"client": "client5", "Shelf_1": "A", "Quantity_1": 23, "Shelf_2": "B", "Quantity_2": 9, "Shelf_3": "C", "Quantity_3": 11, "Shelf_4": "D", "Quantity_4": 12},
	    {"client": "client6", "Shelf_1": "A", "Quantity_1": 8, "Shelf_2": "B", "Quantity_2": 11, "Shelf_3": "C", "Quantity_3": 3, "Shelf_4": "D", "Quantity_4": 13},
	    {"client": "client7", "Shelf_1": "A", "Quantity_1": 24, "Shelf_2": "B", "Quantity_2": 23, "Shelf_3": "C", "Quantity_3": 20, "Shelf_4": "D", "Quantity_4": 9},
       	    {"client": "client8", "Shelf_1": "A", "Quantity_1": 21, "Shelf_2": "B", "Quantity_2": 14, "Shelf_3": "C", "Quantity_3": 4, "Shelf_4": "D", "Quantity_4": 7},
	    {"client": "client9", "Shelf_1": "A", "Quantity_1": 21, "Shelf_2": "B", "Quantity_2": 2, "Shelf_3": "C", "Quantity_3": 15, "Shelf_4": "D", "Quantity_4": 18},
	    {"client": "client10", "Shelf_1": "A", "Quantity_1": 2, "Shelf_2": "B", "Quantity_2": 21, "Shelf_3": "C", "Quantity_3": 6, "Shelf_4": "D", "Quantity_4": 15},
	    {"client": "client11", "Shelf_1": "A", "Quantity_1": 22, "Shelf_2": "B", "Quantity_2": 25, "Shelf_3": "C", "Quantity_3": 9, "Shelf_4": "D", "Quantity_4": 9},
	    {"client": "client12", "Shelf_1": "A", "Quantity_1": 22, "Shelf_2": "B", "Quantity_2": 23, "Shelf_3": "C", "Quantity_3": 17, "Shelf_4": "D", "Quantity_4": 10},
	    {"client": "client13", "Shelf_1": "A", "Quantity_1": 22, "Shelf_2": "B", "Quantity_2": 5, "Shelf_3": "C", "Quantity_3": 21, "Shelf_4": "D", "Quantity_4": 17},
	    {"client": "client14", "Shelf_1": "A", "Quantity_1": 5, "Shelf_2": "B", "Quantity_2": 22, "Shelf_3": "C", "Quantity_3": 25, "Shelf_4": "D", "Quantity_4": 18},
	    {"client": "client15", "Shelf_1": "A", "Quantity_1": 3, "Shelf_2": "B", "Quantity_2": 23, "Shelf_3": "C", "Quantity_3": 23, "Shelf_4": "D", "Quantity_4": 1},
	    {"client": "client16", "Shelf_1": "A", "Quantity_1": 6, "Shelf_2": "B", "Quantity_2": 13, "Shelf_3": "C", "Quantity_3": 12, "Shelf_4": "D", "Quantity_4": 8},
	    {"client": "client17", "Shelf_1": "A", "Quantity_1": 6, "Shelf_2": "B", "Quantity_2": 21, "Shelf_3": "C", "Quantity_3": 18, "Shelf_4": "D", "Quantity_4": 19},
	    {"client": "client18", "Shelf_1": "A", "Quantity_1": 9, "Shelf_2": "B", "Quantity_2": 14, "Shelf_3": "C", "Quantity_3": 8, "Shelf_4": "D", "Quantity_4": 10},
	    {"client": "client19", "Shelf_1": "A", "Quantity_1": 9, "Shelf_2": "B", "Quantity_2": 2, "Shelf_3": "C", "Quantity_3": 7, "Shelf_4": "D", "Quantity_4": 21},
	    {"client": "client20", "Shelf_1": "A", "Quantity_1": 18, "Shelf_2": "B", "Quantity_2": 1, "Shelf_3": "C", "Quantity_3": 24, "Shelf_4": "D", "Quantity_4": 20},
	    {"client": "client21", "Shelf_1": "A", "Quantity_1": 16, "Shelf_2": "B", "Quantity_2": 24, "Shelf_3": "C", "Quantity_3": 13, "Shelf_4": "D", "Quantity_4": 7},
	    {"client": "client22", "Shelf_1": "A", "Quantity_1": 1, "Shelf_2": "B", "Quantity_2": 4, "Shelf_3": "C", "Quantity_3": 6, "Shelf_4": "D", "Quantity_4": 14},
	    {"client": "client23", "Shelf_1": "A", "Quantity_1": 5, "Shelf_2": "B", "Quantity_2": 19, "Shelf_3": "C", "Quantity_3": 17, "Shelf_4": "D", "Quantity_4": 9},
	    {"client": "client24", "Shelf_1": "A", "Quantity_1": 7, "Shelf_2": "B", "Quantity_2": 14, "Shelf_3": "C", "Quantity_3": 14, "Shelf_4": "D", "Quantity_4": 11},
	    {"client": "client25", "Shelf_1": "A", "Quantity_1": 21, "Shelf_2": "B", "Quantity_2": 11, "Shelf_3": "C", "Quantity_3": 16, "Shelf_4": "D", "Quantity_4": 3},
	    {"client": "client26", "Shelf_1": "A", "Quantity_1": 12, "Shelf_2": "B", "Quantity_2": 2, "Shelf_3": "C", "Quantity_3": 19, "Shelf_4": "D", "Quantity_4": 10},
	    {"client": "client27", "Shelf_1": "A", "Quantity_1": 13, "Shelf_2": "B", "Quantity_2": 9, "Shelf_3": "C", "Quantity_3": 25, "Shelf_4": "D", "Quantity_4": 9},
	    {"client": "client28", "Shelf_1": "A", "Quantity_1": 19, "Shelf_2": "B", "Quantity_2": 23, "Shelf_3": "C", "Quantity_3": 5, "Shelf_4": "D", "Quantity_4": 7},
	    {"client": "client29", "Shelf_1": "A", "Quantity_1": 14, "Shelf_2": "B", "Quantity_2": 18, "Shelf_3": "C", "Quantity_3": 8, "Shelf_4": "D", "Quantity_4": 14},
	    {"client": "client30", "Shelf_1": "A", "Quantity_1": 13, "Shelf_2": "B", "Quantity_2": 8, "Shelf_3": "C", "Quantity_3": 6, "Shelf_4": "D", "Quantity_4": 3},
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

