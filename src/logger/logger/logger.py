"""
central_logger.py

This module implements the CentralLogger class, which is responsible for collecting, storing, and forwarding
log messages from various ROS 2 nodes. The CentralLogger node subscribes to the '/central_logs' topic
and logs messages to a CSV file while also sending them to a web server for remote monitoring.

Classes:
    CentralLogger(Node): A ROS 2 node that listens for log messages, stores them locally, and forwards them to a web server.

Functions:
 __-init__(): Initializes the logger node, sets up logging directory and CSV file.
   -log_callback(msg): Processes incoming log messages, stores them in a CSV file, and sends them to the web.
   -send_log_to_web(timestamp, node, log_level, message): Sends log data to a web server via HTTP POST request.
   -main(args=None): Initializes and runs the ROS2 node.
   -main(args=None): Initializes the CentralLogger node and starts the ROS 2 event loop.

CentralLogger Class:
    Methods:
        __init__(): Initializes the CentralLogger node, sets up the log directory, and prepares the log file.
        log_callback(msg): Handles incoming log messages, formats and stores them in a CSV file, and sends them to the web server.
        send_log_to_web(timestamp, node, log_level, message): Sends log data to a remote web server using an HTTP POST request.

Usage:
    This script is intended to be run as a ROS 2 node. It collects logs from other nodes and stores them locally
    while also forwarding them to a remote server.
    To run the node, execute the script directly:
        $ python3 central_logger.py

Dependencies:
    - ROS 2 (rclpy)
    - robot_interfaces.msg (Logs)
    - csv (for log file storage)
    - os (for file and directory handling)
    - requests (for sending log data to a web server)
    - datetime (for timestamp formatting)

Parameters:
    - None explicitly defined.

Topics:
    - /central_logs (Logs): Subscribes to log messages published by other nodes.

Services:
    - None.

Actions:
    - None.

Example:
    To run the CentralLogger node:
        1. Ensure ROS 2 is installed and sourced.
        2. Install dependencies if needed:
            $ pip install requests
        3. Run the script:
            $ python3 central_logger.py
        4. Log files will be stored in the 'logs/' directory.
        5. Logs will also be sent to 'http://localhost:5000/add_log' if the web server is running.
"""

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Logs
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

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(log_dir, f'central_log_{timestamp_str}.csv')

        if not os.path.isfile(self.log_file) or os.stat(self.log_file).st_size == 0:
        # Initialize CSV file with headers if it doesn't exist
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Node', 'Log Level', 'Message'])

        self.log_subscriber = self.create_subscription(Logs, '/central_logs', self.log_callback, 10)
        # Buffer to hold incoming logs
        self.log_buffer = []

        # Second buffer to manage CSV logs (last 500 entries)
        self.csv_buffer = deque(maxlen=500)

        # Load existing logs into the CSV buffer
        if os.path.isfile(self.log_file):
            with open(self.log_file, mode='r', newline='') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header
                for row in reader:
                    self.csv_buffer.append(tuple(row))

        # Create a timer to process logs every 5 seconds
        self.timer = self.create_timer(5.0, self.process_logs)

        self.get_logger().info("Central Logger Node started, listening for logs...")

    def log_callback(self, msg):
        """
        Callback function for the log subscriber.
        Adds incoming logs to the buffer.
        """
        try:
            timestamp_sec = msg.timestamp.sec
            formatted_time = datetime.fromtimestamp(timestamp_sec).strftime('%Y-%m-%d %H:%M:%S')

            node_name = msg.node_name
            log_level = msg.log_level
            message = msg.message

            self.log_buffer.append((formatted_time, node_name, log_level, message))
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

