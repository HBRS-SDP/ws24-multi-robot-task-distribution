import os
from flask import Flask, render_template
from flask_socketio import SocketIO
import rclpy
from std_msgs.msg import String
from rclpy.node import Node

# Absolute path to the templates directory
template_folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')

# Initialize Flask app with the absolute template folder path
app = Flask(__name__, template_folder=template_folder_path)
socketio = SocketIO(app)

# Inventory and Robot Status data (dummy placeholders)
inventory_data = "No data yet"
robot_status = "No status yet"

# ROS 2 Node to publish data from topics
class DataPublisherNode(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.inventory_subscriber = self.create_subscription(
            String, '/inventory_data', self.inventory_callback, 10)
        self.robot_status_subscriber = self.create_subscription(
            String, '/robot_status', self.robot_status_callback, 10)

    def inventory_callback(self, msg):
        global inventory_data
        inventory_data = msg.data
        socketio.emit('inventory_update', {'data': inventory_data})  # Send updated data to web page

    def robot_status_callback(self, msg):
        global robot_status
        robot_status = msg.data
        socketio.emit('robot_status_update', {'data': robot_status})  # Send updated data to web page


@app.route('/')
def index():
    return render_template('index.html')  # Serve the HTML page

# Initialize ROS 2 and Flask
def run_ros_flask():
    # Initialize ROS 2 node
    rclpy.init()
    node = DataPublisherNode()

    # Start the Flask server
    socketio.run(app, host='0.0.0.0', port=5000)

    rclpy.spin(node)

if __name__ == '__main__':
    run_ros_flask()

