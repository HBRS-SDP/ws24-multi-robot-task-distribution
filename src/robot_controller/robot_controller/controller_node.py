import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')  # Corrected initialization
        self.subscription = self.create_subscription(
            String,
            'task_topic',  # The topic the node listens to
            self.complete_task,  # Callback function to process messages
            10  # QoS (Quality of Service) depth
        )
        self.get_logger().info("Controller Node is ready.")

    def complete_task(self, msg):
        # Log the received task
        self.get_logger().info(f"Received task: {msg.data}")
        
        # Here you can add your task completion logic (e.g., collecting items from shelves)
        self.get_logger().info("Collecting items from the shelves...")
        self.get_logger().info("Task completed successfully.")

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)  # Keep the node alive to process incoming messages
    controller_node.destroy_node()  # Cleanup
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()
