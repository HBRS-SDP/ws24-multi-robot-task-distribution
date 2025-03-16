import csv
import os
import requests
from datetime import datetime
from collections import deque
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

        self.log_queue = deque()
        self.lock = threading.Lock()

        self.log_thread = threading.Thread(target=self.process_logs, daemon=True)
        self.log_thread.start()

    def log_callback(self, msg):
        """Callback function for log messages."""
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            node_name, log_level, message = msg.data.split('|', 2)
            log_entry = (timestamp, node_name, log_level, message)

            with self.lock:
                self.log_queue.append(log_entry)

        except Exception as e:
            self.get_logger().error(f"Error processing log message: {e}")

    def process_logs(self):
        """Process log messages in a separate thread."""
        while rclpy.ok():
            if self.log_queue:
                with self.lock:
                    log_entry = self.log_queue.popleft()
                self.write_log(log_entry)
                self.send_log_to_web(*log_entry)
            else:
                rclpy.spin_once(self, timeout_sec=0.1)

    def write_log(self, log_entry):
        """Write log message to a file."""
        try:
            with open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(log_entry)
        except Exception as e:
            self.get_logger().error(f"Error writing log to file: {e}")

    def send_log_to_web(self, timestamp, node, log_level, message):
        """Send log message to a web service."""
        try:
            log_data = {
                "timestamp": timestamp,
                "node": node,
                "log_level": log_level,
                "message": message
            }
            response = requests.post("http://localhost:5000/add_log", json=log_data)
            if response.status_code != 200:
                self.get_logger().error(f"Failed to send log to web: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending log to web: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CentralLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()