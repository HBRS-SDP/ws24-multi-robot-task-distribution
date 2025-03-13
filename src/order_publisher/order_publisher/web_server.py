from flask import Flask, render_template, request, jsonify
import rclpy
from robot_interfaces.msg import Order
from rclpy.node import Node
from rosbridge_websocket import RosBridgeClient

# Initialize Flask app
app = Flask(__name__)

# Initialize ROS 2 node
rclpy.init()
rosbridge = RosBridgeClient("ws://localhost:9090")  # Change if using a different host/port
rosbridge.connect()

# Create ROS 2 publisher
publisher = rosbridge.create_publisher("/order_requests", Order)

@app.route('/')
def index():
    """Render the HTML form to input shelf_id and quantity."""
    return render_template('index.html')

@app.route('/submit_order', methods=['POST'])
def submit_order():
    """Handle order submission and publish the order to ROS 2."""
    shelf_id = int(request.form['shelf_id'])
    quantity = int(request.form['quantity'])

    # Create the Order message
    order_msg = Order()
    order_msg.shelf_id = shelf_id
    order_msg.quantity = quantity

    # Publish the order message to ROS 2
    publisher.publish(order_msg)

    return jsonify({'status': 'success', 'message': f'Order for shelf {shelf_id} with quantity {quantity} sent successfully.'})

if __name__ == '__main__':
    # Run Flask app in a separate thread and start ROS 2 spin
    app.run(debug=True, use_reloader=False)
    rclpy.spin(rosbridge)  # Keep ROS 2 spin loop running

