import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
import requests
from datetime import datetime

class CentralLogger(Node):
    def __init__(self):
        super().__init__('central_logger')

        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.log_file = os.path.join(log_dir, 'central_log.csv')

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

            self.send_log_to_web(timestamp, node_name, log_level, message)

        except Exception as e:
            self.get_logger().error(f"Error processing log message: {e}")

    def send_log_to_web(self, timestamp, node, log_level, message):
        
        try:
            log_data = {
                "timestamp": timestamp,
                "node": node,
                "log_level": log_level,
                "message": message
            }
            response = requests.post("http://localhost:5000/add_log", json=log_data)
            if response.status_code == 200:
                self.get_logger().info("Log successfully sent to web server")
            else:
                self.get_logger().error(f"Failed to send log: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending log to web: {e}")

def main(args=None):
    rclpy.init(args=args)
    logger = CentralLogger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

