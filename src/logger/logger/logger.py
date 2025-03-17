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
