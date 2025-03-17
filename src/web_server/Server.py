#!/usr/bin/env python3

from flask import Flask, render_template, request, redirect, url_for, jsonify
import rclpy
from robot_interfaces.msg import Order, FleetStatus, Product
from robot_interfaces.srv import GetShelfList, InventoryUpdate
from rclpy.node import Node
import threading
from datetime import datetime
import csv
import os
import time
import webbrowser
from rclpy.executors import SingleThreadedExecutor


rclpy.init()
node = Node('web_server')

log_dir = "published_orders"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

ORDERS_CSV_FILE = os.path.join(log_dir, "published_orders.csv")

publisher = node.create_publisher(Order, '/order_requests', 10)
shelf_list_client = node.create_client(GetShelfList, '/get_shelf_list')
inventory_update_client = node.create_client(InventoryUpdate, '/update_inventory')

app = Flask(__name__)

orders = []
available_shelves = []
shelf_to_product = {}
robot_status_data = {}
logs_list = []
    
def fleet_status_callback(msg):
    global robot_status_data
    robot_status_data = {}
    for robot in msg.robot_status_list:
        robot_status_data[robot.robot_id] = {
            "battery_level": robot.battery_level,
            "status": "idle" if robot.is_available else "occupied"
        }

node.create_subscription(FleetStatus, 'fleet_status', fleet_status_callback, 10)

def get_shelf_list_callback(response):
    global available_shelves, shelf_to_product
    available_shelves = [shelf for shelf in response.shelf_status_list if shelf.current_inventory > 0]
    shelf_to_product = {shelf.shelf_id: shelf.product for shelf in available_shelves}

def get_shelf_data():
    """Fetch shelf list data when needed instead of continuously polling."""
    if shelf_list_client.wait_for_service(timeout_sec=2.0):
        print("[DEBUG] Requesting latest shelf data...")
        request = GetShelfList.Request()
        future = shelf_list_client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin_once(timeout_sec=2.0)  # Process a single event
        executor.remove_node(node)

        response = future.result()
        response = future.result()

        if response:
            global available_shelves, shelf_to_product
            available_shelves = []
            shelf_to_product = {}

            for shelf in response.shelf_status_list:
                if shelf.current_inventory > 0:  # Only include stocked shelves
                    available_shelves.append(shelf)
                    shelf_to_product[shelf.shelf_id] = shelf.product
        else:
            print("[DEBUG] No response from service!")
    else:
        print("[ERROR] Shelf list service unavailable!")

def log_order_to_csv(order):
    """Logs the published order details into a CSV file."""
    file_exists = os.path.isfile(ORDERS_CSV_FILE)

    with open(ORDERS_CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write header only if file is newly created
        if not file_exists:
            writer.writerow(["Order ID", "Shelf ID", "Quantity", "Timestamp"])

        for shelf in order['shelves']:
            writer.writerow([order['order_id'], shelf['shelf_id'], shelf['quantity'], order['timestamp']])

@app.route('/')
def index(): 
    global available_shelves, shelf_to_product

    print("[DEBUG] Fetching shelf data before rendering page...")
    get_shelf_data()  # Fetch shelf data before rendering

    print(f"[DEBUG] Sending to HTML: {len(available_shelves)} shelves")
    return render_template('index.html', available_shelves=available_shelves, shelf_to_product=shelf_to_product, orders=orders, logs=logs_list)

@app.route('/add_log', methods=['POST'])
def add_log():
    log_data = request.json
    logs_list.append(log_data)  # Store in memory

    return jsonify({"success": True}), 200

@app.route('/get_logs', methods=['GET'])
def get_logs():
    return jsonify(logs_list)

@app.route('/get_inventory')
def get_inventory():
    global available_shelves

    get_shelf_data()

    inventory_data = [
        {
            "shelf_id": shelf.shelf_id,
            "product": shelf.product,
            "capacity": shelf.shelf_capacity,
            "current_inventory": shelf.current_inventory
        }
        for shelf in available_shelves
    ]

    return jsonify(inventory_data)

@app.route('/get_robot_status')
def get_robot_status():
    return jsonify(robot_status_data)
    
@app.route('/update_inventory', methods=['POST'])
def update_inventory():
    try:
        
        request_data = request.get_json()  

        if not request_data or 'shelf_id' not in request_data or 'new_inventory' not in request_data:
             return jsonify({"success": False, "error": "Invalid request data"}), 400

        shelf_id = int(request_data['shelf_id'])
        new_inventory = int(request_data['new_inventory'])
        
        request_msg = InventoryUpdate.Request()
        request_msg.shelf_id = shelf_id
        request_msg.new_inventory = new_inventory

        future = inventory_update_client.call_async(request_msg)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()

        if response.success:
            return jsonify({"success": True, "message": f"Inventory updated for shelf {shelf_id} to {new_inventory}"}), 200
        else:
            return jsonify({"success": False, "error": "Shelf ID not found in shared memory"}), 404

    except Exception as e:
        return jsonify({"success": False, "error": f"Internal Server Error: {str(e)}"}), 500

@app.route('/submit_order', methods=['POST'])
def submit_order():
    
    global orders, available_shelves, shelf_to_product

    get_shelf_data()
    
    shelf_ids = request.form.getlist('shelf_id[]')
    quantities = request.form.getlist('quantity[]')

    inventory_map = {shelf.shelf_id: shelf.current_inventory for shelf in available_shelves}

    shelves = []
    error_messages = []
    
    for shelf_id, quantity in zip(shelf_ids, quantities):
        try:
            shelf_id, quantity = int(shelf_id), int(quantity)

            if shelf_id > 0 and quantity > 0:
                
                if shelf_id in inventory_map and quantity <= inventory_map[shelf_id]:
                    shelves.append({'shelf_id': shelf_id, 'quantity': quantity})
                else:
                    product_name = shelf_to_product.get(shelf_id, "Unknown Product")
                    error_messages.append(f"Only {inventory_map.get(shelf_id, 0)} items available for product '{product_name}'!")
                   
        except ValueError:
            continue

    if error_messages:
        return render_template('index.html', 
                               available_shelves=available_shelves, 
                               shelf_to_product=shelf_to_product, 
                               orders=orders, 
                               error_message="<br>".join(error_messages))  # Joining errors with line breaks

    if not shelves:
        return render_template('index.html', 
                               available_shelves=available_shelves, 
                               shelf_to_product=shelf_to_product, 
                               orders=orders, 
                               error_message="No valid products in the order!")

    # Create and publish order
    order_id = len(orders) + 1
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    order = {'order_id': order_id, 'shelves': shelves, 'timestamp': timestamp}
    orders.append(order)

    log_order_to_csv(order)

    order_msg = Order()
    order_msg.order_id = order_id
    for shelf in shelves:
        product = Product()
        product.shelf_id = shelf['shelf_id']
        product.quantity = shelf['quantity']
        order_msg.product_list.append(product)

    publisher.publish(order_msg)

    return render_template('index.html', 
                           available_shelves=available_shelves, 
                           shelf_to_product=shelf_to_product, 
                           orders=orders, 
                           success_message="Order placed successfully!")

@app.route('/repeat_order/<int:order_id>', methods=['POST'])
def repeat_order(order_id):
    global orders, available_shelves, shelf_to_product

    order_to_repeat = next((order for order in orders if order['order_id'] == order_id), None)

    if not order_to_repeat:
        return jsonify({"success": False, "error": "Order not found"}), 404

    new_order_id = len(orders) + 1
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    new_order = {
        'order_id': new_order_id,
        'shelves': order_to_repeat['shelves'],  
        'timestamp': timestamp
    }

    orders.append(new_order)
    log_order_to_csv(new_order)

    order_msg = Order()
    for shelf in new_order['shelves']:
        product = Product()
        product.shelf_id = shelf['shelf_id']
        product.quantity = shelf['quantity']
        order_msg.product_list.append(product)

    publisher.publish(order_msg)  

    print(f"[DEBUG] Repeated Order {new_order_id} published: {new_order}")

    return redirect(url_for('index'))

def run_flask():
    url = "http://127.0.0.1:5000/"
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False))
    flask_thread.start()
    time.sleep(3)
    webbrowser.open(url)

def run_ros(): 
    global node
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()  # Keep ROS node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    flask_thread = threading.Thread(target=run_flask)
    ros_thread = threading.Thread(target=run_ros)

    flask_thread.start()
    ros_thread.start()

    flask_thread.join()
    ros_thread.join()