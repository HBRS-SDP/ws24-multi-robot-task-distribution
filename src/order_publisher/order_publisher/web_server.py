from flask import Flask, render_template, request, redirect, url_for, jsonify
import rclpy
from robot_interfaces.msg import Order
from rclpy.node import Node
import threading
from datetime import datetime
import csv

# Initialize ROS node
rclpy.init()
node = Node('order_publisher')

# Create a ROS publisher
publisher = node.create_publisher(Order, '/order_requests', 10)

# Initialize Flask app
app = Flask(__name__)

# Store orders in memory (this would be better in a database for persistence)
orders = []

def read_logs():
    logs = []
    try:
        with open('fleet_manager_log.csv', mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                logs.append(row)
    except FileNotFoundError:
        print("Log file not found.")
    return logs

@app.route('/')
def index():
    logs = read_logs()  # Fetch logs from CSV
    print(f"Logs being passed to template: {logs}")
    return render_template('index.html', orders=orders, logs=logs)
    
@app.route('/get_logs')
def get_logs():
    logs = read_logs()  # Fetch logs from CSV
    return jsonify(logs)  # Return logs in JSON format

@app.route('/submit_order', methods=['POST'])
def submit_order():
    # Get shelf_ids and quantities from the form
    shelf_ids = request.form.getlist('shelf_id[]')
    quantities = request.form.getlist('quantity[]')

    # Create a list of shelves for the order
    shelves = []
    for shelf_id, quantity in zip(shelf_ids, quantities):
        shelves.append({'shelf_id': int(shelf_id), 'quantity': int(quantity)})

    # Generate timestamp (No order_id is used)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Create an order dictionary (without order_id)
    order = {
        'shelves': shelves,
        'timestamp': timestamp
    }

    # Append order to the orders list
    orders.append(order)

    # Publish the order to the ROS topic
    order_msg = Order()
    for shelf in shelves:
        order_msg.shelf_id = shelf['shelf_id']
        order_msg.quantity = shelf['quantity']
        publisher.publish(order_msg)

    # Log the published order
    node.get_logger().info(f"Published Order: {order}")

    # Redirect back to the index page
    return redirect(url_for('index'))

@app.route('/delete_order/<int:order_id>', methods=['POST'])
def delete_order(order_id):
    global orders
    orders = [order for order in orders if order['order_id'] != order_id]
    return redirect(url_for('index'))

# Function to run Flask in a thread
def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

# Function to run ROS node
def run_ros():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Start the Flask and ROS nodes in separate threads
    flask_thread = threading.Thread(target=run_flask)
    ros_thread = threading.Thread(target=run_ros)

    # Start both threads
    flask_thread.start()
    ros_thread.start()

    # Wait for both threads to finish
    flask_thread.join()
    ros_thread.join()

