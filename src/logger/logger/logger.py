import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
import requests
from datetime import datetime
from collections import deque

class CentralLogger(Node):
    def __init__(self):
        super().__init__('central_logger')

        # Directory and file setup
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.log_file = os.path.join(log_dir, 'central_log.csv')

        # Initialize CSV file with headers if it doesn't exist
        if not os.path.isfile(self.log_file):
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Node', 'Log Level', 'Message'])

        # Buffer to hold incoming logs
        self.log_buffer = []

        # Second buffer to manage CSV logs (last 2000 entries)
        self.csv_buffer = deque(maxlen=2000)

        # Load existing logs into the CSV buffer
        if os.path.isfile(self.log_file):
            with open(self.log_file, mode='r', newline='') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header
                for row in reader:
                    self.csv_buffer.append(tuple(row))

        # Create a subscription to the log topic
        self.log_subscriber = self.create_subscription(String, '/central_logs', self.log_callback, 10)

        # Create a timer to process logs every 5 seconds
        self.timer = self.create_timer(5.0, self.process_logs)

        self.get_logger().info("Central Logger Node started, listening for logs...")

    def log_callback(self, msg):
        """
        Callback function for the log subscriber.
        Adds incoming logs to the buffer.
        """
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            node_name, log_level, message = msg.data.split('|', 2)
            self.log_buffer.append((timestamp, node_name, log_level, message))
        except Exception as e:
            self.get_logger().error(f"Error processing log message: {e}")

    def process_logs(self):
        """
        Processes logs in the buffer every 5 seconds.
        Writes logs to the CSV file and sends them to the web GUI.
        """
        if not self.log_buffer:
            return  # No logs to process

        try:
            # Add new logs to the CSV buffer
            self.csv_buffer.extend(self.log_buffer)

            # Write the CSV buffer to the CSV file
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Node', 'Log Level', 'Message'])  # Write header
                writer.writerows(self.csv_buffer)

            # Send logs to the web GUI
            self.send_logs_to_web(self.log_buffer)

            # Clear the main buffer after processing
            self.log_buffer.clear()

            self.get_logger().info(f"Processed {len(self.log_buffer)} logs.")
        except Exception as e:
            self.get_logger().error(f"Error processing logs: {e}")

    def send_logs_to_web(self, logs):
        """
        Sends logs to the web GUI.
        """
        try:
            log_data = [{
                "timestamp": log[0],
                "node": log[1],
                "log_level": log[2],
                "message": log[3]
            } for log in logs]

            response = requests.post("http://localhost:5000/add_log", json=log_data)
            if response.status_code == 200:
                self.get_logger().info("Logs successfully sent to web server")
            else:
                self.get_logger().error(f"Failed to send logs: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending logs to web: {e}")

def main(args=None):
    rclpy.init(args=args)
    logger = CentralLogger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()