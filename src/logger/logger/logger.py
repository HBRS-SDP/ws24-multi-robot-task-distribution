import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
from datetime import datetime

class CentralLogger(Node):
    def __init__(self):
        super().__init__('central_logger')

        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.log_file = os.path.join(log_dir, 'central_log.csv')

        # Create CSV file with headers if not exists
        if not os.path.isfile(self.log_file):
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Node', 'Log Level', 'Message'])

        self.log_subscriber = self.create_subscription(String, '/central_logs', self.log_callback, 10)
        self.get_logger().info("Central Logger Node started, listening for logs...")

    def log_callback(self, msg):
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            node_name, log_level, message = msg.data.split('|', 2)

            with open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, node_name, log_level, message])

            self.get_logger().info(f"Logged from {node_name}: {message}")

        except Exception as e:
            self.get_logger().error(f"Error processing log message: {e}")

def main(args=None):
    rclpy.init(args=args)
    logger = CentralLogger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

