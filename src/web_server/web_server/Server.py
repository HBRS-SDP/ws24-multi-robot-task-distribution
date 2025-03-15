from flask import Flask, render_template, request, redirect, url_for, jsonify
import rclpy
from robot_interfaces.msg import Order, FleetStatus, Product
from robot_interfaces.srv import GetShelfList, InventoryUpdate
from rclpy.node import Node
import threading
from datetime import datetime
import csv
import time

rclpy.init()
node = Node('web_server')

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
    
    #print(f"Updated Shelves: {available_shelves}")
    #print(f"Shelf to Product Mapping: {shelf_to_product}")


def fetch_shelf_data():
    while rclpy.ok():
        if shelf_list_client.wait_for_service(timeout_sec=1.0):
            print("[DEBUG] Service available. Requesting shelf data...")

            request = GetShelfList.Request()
            future = shelf_list_client.call_async(request)

            # Wait for the service response
            rclpy.spin_until_future_complete(node, future)
            response = future.result()

            if response:
                global available_shelves, shelf_to_product
                available_shelves = []
                shelf_to_product = {}

                for shelf in response.shelf_status_list:
                    if shelf.current_inventory > 0:  # Only include shelves with stock
                        available_shelves.append(shelf)
                        shelf_to_product[shelf.shelf_id] = shelf.product

            else:
                print("[DEBUG] No response from service!")

            time.sleep(2)
        else:
            print("Service not available, retrying...")

@app.route('/')
def index(): 
    global available_shelves, shelf_to_product

    attempts = 5
    while not available_shelves and attempts > 0:
        print("[DEBUG] Waiting for shelves to be fetched...")
        time.sleep(2)
        attempts -= 1 
    print(f"Sending to HTML: {available_shelves}")
    return render_template('index.html', available_shelves=available_shelves, shelf_to_product=shelf_to_product, orders=orders, logs=logs_list)

@app.route('/add_log', methods=['POST'])
def add_log():
    """Receive logs from Central Logger and store them."""
    log_data = request.json
    logs_list.append(log_data)  # Store in memory

    # Save log to CSV file
    with open("web_logs.csv", mode="a", newline='') as file:
        writer = csv.writer(file)
        writer.writerow([log_data["timestamp"], log_data["node"], log_data["log_level"], log_data["message"]])

    return jsonify({"success": True}), 200

@app.route('/get_logs', methods=['GET'])
def get_logs():
    
    return jsonify(logs_list)

@app.route('/get_inventory')
def get_inventory():
    global available_shelves

    request = GetShelfList.Request()
    future = shelf_list_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    if response:
        available_shelves = response.shelf_status_list
        inventory_data = [
            {
                "shelf_id": shelf.shelf_id,
                "product": shelf.product,
                "capacity": shelf.shelf_capacity,
                "current_inventory": shelf.current_inventory
            }
            for shelf in available_shelves
        ]
    else:
        inventory_data = []

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
    
    shelf_ids = request.form.getlist('shelf_id[]')
    quantities = request.form.getlist('quantity[]')

    product_list = []
    shelves = []
    for shelf_id, quantity in zip(shelf_ids, quantities):
        try:
            shelf_id = int(shelf_id)
            quantity = int(quantity)

            if shelf_id > 0 and quantity > 0:
                shelves.append({'shelf_id': shelf_id, 'quantity': quantity})
                product = Product()
                product.shelf_id = shelf_id
                product.quantity = quantity
                product_list.append(product)
            else:
                print(f"Ignored invalid shelf ID {shelf_id} or quantity {quantity}")
        except ValueError:
            print(f"Error: Invalid data for shelf_id {shelf_id}")
            continue
            
    if not shelves:
        print("No valid shelves in order submission! Rejecting order.")
        return "Error: No valid shelves selected!", 400

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    order_id = len(orders) + 1

    order = {'order_id': order_id, 'shelves': shelves, 'timestamp': timestamp}
    orders.append(order)

    order_msg = Order()
    order_msg.product_list = product_list
    publisher.publish(order_msg)

    print(f"Published Order: {order_msg}")
    
    save_order_to_csv(order)

    # Log the published order
    #node.get_logger().info(f"Published Order: {order}")

    return redirect(url_for('index'))

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

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def run_ros():
    fetch_shelf_data() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    flask_thread = threading.Thread(target=run_flask)
    ros_thread = threading.Thread(target=run_ros)

    flask_thread.start()
    ros_thread.start()

    flask_thread.join()
    ros_thread.join()
