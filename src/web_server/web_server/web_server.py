from flask import Flask, render_template, request, redirect, url_for, jsonify
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order, FleetStatus
from robot_interfaces.srv import GetShelfList, InventoryUpdate
import threading
from datetime import datetime
import csv
import time


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')

        # Initialize Flask app
        self.app = Flask(__name__)
        self.orders = []
        self.available_shelves = []
        self.shelf_to_product = {}
        self.robot_status_data = {}

        # ROS2 Publishers and Clients
        self.publisher = self.create_publisher(Order, '/order_requests', 10)
        self.shelf_list_client = self.create_client(GetShelfList, '/get_shelf_list')
        self.inventory_update_client = self.create_client(InventoryUpdate, '/update_inventory')

        # Subscribe to fleet status updates
        self.create_subscription(FleetStatus, 'fleet_status', self.fleet_status_callback, 10)

        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.start()

        # Start shelf data fetch in a separate thread
        self.shelf_fetch_thread = threading.Thread(target=self.fetch_shelf_data)
        self.shelf_fetch_thread.start()

    def fleet_status_callback(self, msg):
        """Updates the robot fleet status based on ROS messages."""
        self.robot_status_data = {
            robot.robot_id: {
                "battery_level": robot.battery_level,
                "status": "idle" if robot.is_available else "occupied"
            }
            for robot in msg.robot_status_list
        }

    def fetch_shelf_data(self):
        """Continuously fetches shelf data from ROS2 service."""
        while rclpy.ok():
            if self.shelf_list_client.wait_for_service(timeout_sec=1.0):
                request = GetShelfList.Request()
                future = self.shelf_list_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()

                if response:
                    self.available_shelves = [
                        shelf for shelf in response.shelf_status_list if shelf.current_inventory > 0
                    ]
                    self.shelf_to_product = {shelf.shelf_id: shelf.product for shelf in self.available_shelves}
            time.sleep(2)

    def read_logs(self):
        """Reads log file data."""
        logs = []
        try:
            with open('fleet_manager_log.csv', mode='r') as file:
                reader = csv.DictReader(file)
                logs = list(reader)
        except FileNotFoundError:
            self.get_logger().warn("Log file not found.")
        return logs

    def run_flask(self):
        """Starts the Flask web server."""

        @self.app.route('/')
        def index():
            logs = self.read_logs()

            # Wait up to 10 seconds for shelves to be available
            attempts = 5
            while not self.available_shelves and attempts > 0:
                self.get_logger().info("[DEBUG] Waiting for shelves to be fetched...")
                time.sleep(2)
                attempts -= 1

            return render_template(
                'index.html',
                available_shelves=self.available_shelves,
                shelf_to_product=self.shelf_to_product,
                orders=self.orders,
                logs=logs
            )

        @self.app.route('/get_logs')
        def get_logs():
            return jsonify(self.read_logs())

        @self.app.route('/get_inventory')
        def get_inventory():
            request = GetShelfList.Request()
            future = self.shelf_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            inventory_data = [
                {
                    "shelf_id": shelf.shelf_id,
                    "product": shelf.product,
                    "capacity": shelf.shelf_capacity,
                    "current_inventory": shelf.current_inventory
                }
                for shelf in response.shelf_status_list
            ] if response else []

            return jsonify(inventory_data)

        @self.app.route('/get_robot_status')
        def get_robot_status():
            return jsonify(self.robot_status_data)

        @self.app.route('/update_inventory', methods=['POST'])
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

                future = self.inventory_update_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()

                if response.success:
                    return jsonify({"success": True, "message": f"✅ Inventory updated for shelf {shelf_id}"}), 200
                else:
                    return jsonify({"success": False, "error": "❌ Shelf ID not found"}), 404
            except Exception as e:
                return jsonify({"success": False, "error": f"Internal Server Error: {str(e)}"}), 500

        @self.app.route('/submit_order', methods=['POST'])
        def submit_order():
            shelf_ids = request.form.getlist('shelf_id[]')
            quantities = request.form.getlist('quantity[]')

            shelves = []
            for shelf_id, quantity in zip(shelf_ids, quantities):
                try:
                    shelf_id, quantity = int(shelf_id), int(quantity)
                    if shelf_id > 0 and quantity > 0:
                        shelves.append({'shelf_id': shelf_id, 'quantity': quantity})
                except ValueError:
                    continue

            if not shelves:
                return "Error: No valid shelves selected!", 400

            order = {
                'order_id': len(self.orders) + 1,
                'shelves': shelves,
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.orders.append(order)

            order_msg = Order()
            for shelf in shelves:
                order_msg.shelf_id_list.append(shelf['shelf_id'])
                order_msg.quantity_list.append(shelf['quantity'])

            self.publisher.publish(order_msg)
            self.save_order_to_csv(order)

            return redirect(url_for('index'))

        @self.app.route('/delete_order/<int:order_id>', methods=['POST'])
        def delete_order(order_id):
            self.orders = [order for order in self.orders if order['order_id'] != order_id]
            return redirect(url_for('index'))

        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

    def save_order_to_csv(self, order):
        """Saves order to CSV file."""
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


def main():
    rclpy.init()
    node = WebServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
