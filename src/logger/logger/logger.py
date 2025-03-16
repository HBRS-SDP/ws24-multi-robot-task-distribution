import csv
import os
import requests
from datetime import datetime
from collections import deque
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import aiofiles
import asyncio

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

        self.log_queue = deque(maxlen=10000)  # Limit the queue size
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

    async def write_logs_async(self, log_entries):
        """Write log messages to a file asynchronously."""
        try:
            async with aiofiles.open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                await writer.writerows(log_entries)
        except Exception as e:
            self.get_logger().error(f"Error writing logs to file: {e}")

    def send_logs_to_web(self, log_entries):
        """Send log messages to a web service."""
        try:
            log_data = [{"timestamp": entry[0], "node": entry[1], "log_level": entry[2], "message": entry[3]} for entry in log_entries]
            response = requests.post("http://localhost:5000/add_log", json=log_data)
            if response.status_code != 200:
                self.get_logger().error(f"Failed to send logs to web: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending logs to web: {e}")

    def process_logs(self):
        """Process log messages in a separate thread."""
        buffer = []
        while rclpy.ok():
            time.sleep(10)  # Flush buffer every 10 seconds
            with self.lock:
                while self.log_queue:
                    buffer.append(self.log_queue.popleft())

            if buffer:
                asyncio.run(self.write_logs_async(buffer))
                self.send_logs_to_web(buffer)
                buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = CentralLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()