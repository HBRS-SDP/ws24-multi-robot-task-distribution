from flask import Flask, render_template, request, redirect, url_for, jsonify
import rclpy
from robot_interfaces.msg import Order
from robot_interfaces.srv import GetShelfList
from rclpy.node import Node
import threading
from datetime import datetime
import csv
import time

# Initialize ROS node
rclpy.init()
node = Node('order_publisher')

# Create a ROS publisher
publisher = node.create_publisher(Order, '/order_requests', 10)

# Create the ROS service client for fetching the shelf list
shelf_list_client = node.create_client(GetShelfList, '/get_shelf_list')

# Initialize Flask app
app = Flask(__name__)

# Store orders in memory (would be better in a database)
orders = []

# Store shelves and products globally
available_shelves = []
shelf_to_product = {}

# 🚀 Read logs from `fleet_manager_log.csv`
def read_logs():
    logs = []
    try:
        with open('fleet_manager_log.csv', mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                logs.append(row)
    except FileNotFoundError:
        print("⚠️ Log file not found.")
    return logs

# 🚀 ROS callback to handle shelf list service response
def get_shelf_list_callback(response):
    global available_shelves, shelf_to_product
    available_shelves = [shelf for shelf in response.shelf_status_list if shelf.current_inventory > 0]
    shelf_to_product = {shelf.shelf_id: shelf.product for shelf in available_shelves}

# 🚀 Fetch shelf data from ROS service
def fetch_shelf_data():
    while rclpy.ok():
        if shelf_list_client.wait_for_service(timeout_sec=1.0):
            request = GetShelfList.Request()
            future = shelf_list_client.call_async(request)
            future.add_done_callback(lambda future: get_shelf_list_callback(future.result()))
            rclpy.spin_once(node)
            time.sleep(2)  # Fetch data every 2 seconds
        else:
            print("❌ Service not available, retrying...")

@app.route('/')
def index():
    logs = read_logs()  # Fetch logs from CSV
    return render_template('index.html', available_shelves=available_shelves, shelf_to_product=shelf_to_product, orders=orders, logs=logs)

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
        try:
            shelf_id = int(shelf_id)
            quantity = int(quantity)
            if shelf_id > 0 and quantity > 0:
                shelves.append({'shelf_id': shelf_id, 'quantity': quantity})
            else:
                print(f"⚠️ Ignored invalid shelf ID {shelf_id} or quantity {quantity}")
        except ValueError:
            print(f"🚨 Error: Invalid data for shelf_id {shelf_id}")
            continue  # Skip bad input

    if not shelves:
        print("🚨 No valid shelves in order submission! Rejecting order.")
        return "Error: No valid shelves selected!", 400  # Return error response

    # Generate timestamp and order ID
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    order_id = len(orders) + 1  # Simple order ID assignment

    # Create order object
    order = {'order_id': order_id, 'shelves': shelves, 'timestamp': timestamp}
    orders.append(order)

    # Publish the order to the ROS topic
    order_msg = Order()
    for shelf in shelves:
        order_msg.shelf_id_list.append(shelf['shelf_id'])
        order_msg.quantity_list.append(shelf['quantity'])
    publisher.publish(order_msg)

    print(f"🛒 Published Order: {order_msg}")

    # Save order to CSV
    save_order_to_csv(order)

    # Redirect back to the index page
    return redirect(url_for('index'))

# 🚀 Save orders to CSV
def save_order_to_csv(order):
    file_name = 'published_orders.csv'
    file_exists = False

    try:
        with open(file_name, mode='r'):
            file_exists = True
    except FileNotFoundError:
        file_exists = False

    if not file_exists:
        with open(file_name, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=['order_id', 'timestamp', 'shelf_id', 'quantity'])
            writer.writeheader()

    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        for shelf in order['shelves']:
            writer.writerow([order['order_id'], order['timestamp'], shelf['shelf_id'], shelf['quantity']])

@app.route('/delete_order/<int:order_id>', methods=['POST'])
def delete_order(order_id):
    global orders
    orders = [order for order in orders if order['order_id'] != order_id]
    return redirect(url_for('index'))

# 🚀 Run Flask in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

# 🚀 Run ROS node in a separate thread
def run_ros():
    fetch_shelf_data()  # Fetch shelf data asynchronously
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Start Flask and ROS in parallel
    flask_thread = threading.Thread(target=run_flask)
    ros_thread = threading.Thread(target=run_ros)

    flask_thread.start()
    ros_thread.start()

    flask_thread.join()
    ros_thread.join()

