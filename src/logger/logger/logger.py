import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Logs
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

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(log_dir, f'central_log_{timestamp_str}.csv')

        if not os.path.isfile(self.log_file) or os.stat(self.log_file).st_size == 0:
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Node', 'Log Level', 'Message'])

        self.log_subscriber = self.create_subscription(Logs, '/central_logs', self.log_callback, 10)
        self.get_logger().info("Central Logger Node started, listening for logs...")

    def log_callback(self, msg):
        try:
            timestamp_sec = msg.timestamp.sec
            formatted_time = datetime.fromtimestamp(timestamp_sec).strftime('%Y-%m-%d %H:%M:%S')

            node_name = msg.node_name
            log_level = msg.log_level
            message = msg.message

            formatted_message = f"[{formatted_time}] [{log_level}] {node_name}: {message}"
            self.get_logger().info(formatted_message)

            with open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([formatted_time, node_name, log_level, message])

            self.send_log_to_web(formatted_time, node_name, log_level, message)

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
