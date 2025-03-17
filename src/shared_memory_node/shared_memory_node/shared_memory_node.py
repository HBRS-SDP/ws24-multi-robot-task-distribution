"""
Shared Memory Node

This node acts as a centralized shared memory module for the warehouse simulation. It manages and provides access to 
data such as robot statuses, shelf inventories, and drop-off locations. The node interacts with other components 
via ROS2 topics, services, and publishers to ensure efficient data sharing and synchronization.

### Responsibilities:
1. **Database Management**:
   - Loads and maintains a database of shelves and robots.
   - Updates inventory and robot statuses dynamically.

2. **Logging**:
   - Publishes logs to the `/central_logs` topic for centralized monitoring.

3. **Service Interactions**:
   - Provides services for querying shelf details, updating inventory, retrieving robot statuses, and getting fleet status.

4. **Subscriptions**:
   - Subscribes to topics such as `/end_order` to update inventory after task completion.
   - Subscribes to `/fleet_status` to update robot statuses.

### Topics:
- **Subscribed**:
  - `/end_order`: Updates inventory and robot statuses when an order is completed.
  - `/fleet_status`: Updates the current status of the robot fleet.

- **Published**:
  - `/central_logs`: Publishes logs for centralized monitoring.

### Services:
- **Provided**:
  - `/shelf_query`: Returns details of a specific shelf.
  - `/update_inventory`: Updates the inventory of a shelf.
  - `/get_robot_state`: Returns the current state of a specific robot.
  - `/get_robot_fleet_status`: Returns the status of the entire robot fleet.
  - `/get_shelf_list`: Returns a list of all shelves and their details.
  - `/get_drop_off_pose`: Returns the drop-off location for completed tasks.

### Key Features:
- Loads inventory data from a CSV file and initializes the database.
- Dynamically updates inventory and robot statuses based on incoming data.
- Provides efficient querying and updating of shared memory data through services.
- Logs all significant events and database changes for monitoring and debugging.

### Execution:
- The node is initialized and spun using ROS2, and it runs until shutdown.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from robot_interfaces.srv import ShelfQuery, InventoryUpdate, GetRobotStatus, GetRobotFleetStatus, GetShelfList, GetPose
from robot_interfaces.msg import ShelfStatus, FleetStatus, Order, Logs
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import json
import os
import csv
from  builtin_interfaces.msg import Time

class SharedMemoryNode(Node):
    def __init__(self):
        super().__init__('shared_memory_node')

        database_dir = get_package_share_directory("shared_memory_node")
        database_name = "shelves_database"
        database_file = os.path.join(database_dir, "databases", database_name + ".csv")
        self.drop_off_pose = Pose()

        # Initialize the database with sample data
        self.database = {"shelves": []}
        self.robots = []
        self.shelves = []

        # Publishers
        self.log_publisher = self.create_publisher(Logs, '/central_logs', 10)

        self.load_inventory(database_file)
        self.database["shelves"] = self.shelves
        self.database["robots"] = self.robots

        # Create a QoS profile for fleet status
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers
        self.order_end_sub = self.create_subscription(Order, '/end_order', self.order_end_callback, 10)
        self.fleet_status_sub = self.create_subscription(FleetStatus, '/fleet_status', self.fleet_status_callback,
                                                         qos_profile=qos_profile)
        

        # Services
        self.database_query_service = self.create_service(
            ShelfQuery, '/shelf_query', self.shelf_query_callback)
        self.inventory_update_service = self.create_service(
            InventoryUpdate, '/update_inventory', self.inventory_update_callback)
        self.robot_state_service = self.create_service(
            GetRobotStatus, '/get_robot_state', self.robot_state_callback)
        self.robot_fleet_service = self.create_service(
            GetRobotFleetStatus, '/get_robot_fleet_status', self.get_fleet_status_callback)
        self.shelf_list_service = self.create_service(
            GetShelfList, '/get_shelf_list', self.get_shelf_list_callback)
        self.get_drop_off_pose_service = self.create_service(
            GetPose, '/get_drop_off_pose', self.get_drop_off_pose_callback)

        self.log_to_central("INFO", "Database Module is ready.")

    def log_to_central(self, level, message):
        """Publishes logs to the central logging topic."""
        log_msg = Logs()
        log_msg.timestamp =  self.get_clock().now().to_msg()
        log_msg.node_name = "Shared Memory"
        log_msg.log_level = level
        log_msg.message = message
        self.log_publisher.publish(log_msg)

    def load_inventory(self, database_file):
        """Reads inventory data from the CSV file and stores it in a dictionary."""

        try:
            with open(database_file, mode='r') as file:
                reader = csv.DictReader(file)
                for idx, row in enumerate(reader):
                    shelf_pose = Pose()
                    shelf_pose.position.x = float(row["X"])
                    shelf_pose.position.y = float(row["Y"])
                    shelf_pose.position.z = float(row["Z"])
                    shelf_pose.orientation.x = float(row["Q_X"])
                    shelf_pose.orientation.y = float(row["Q_Y"])
                    shelf_pose.orientation.z = float(row["Q_Z"])
                    shelf_pose.orientation.w = float(row["Q_W"])

                    if idx == 0:
                        self.drop_off_pose = shelf_pose
                        self.log_to_central("INFO", f"Drop off location loaded with id: {row['id']}")
                    else:
                        # "product" in row and "capacity" in row and "inventory" in row:
                        shelf = ShelfStatus()
                        shelf.shelf_id = int(row["id"])
                        shelf.shelf_location = shelf_pose
                        shelf.product = row["product"]
                        shelf.shelf_capacity = int(row["capacity"])
                        shelf.current_inventory = int(row["inventory"])
                        self.shelves.append(shelf)

            self.log_to_central("INFO", "Inventory successfully loaded from CSV.")
            self.log_database()
        except Exception as e:
            self.log_to_central("ERROR", f"Error loading inventory from CSV: {e}")



    def order_end_callback(self, order):
        """Updates robot status and inventories when it reaches a shelf."""

        if not order.product_list:
            self.log_to_central("WARN", "No product list in order.")
            return

        self.update_inventory_status(order)

        self.log_database()


    def update_inventory_status(self, order):
        if not self.shelves:
            self.log_to_central("INFO", "No shelves available to update inventory.")
            return

        for product in order.product_list:
            for idx, shelf in enumerate(self.shelves):
                if shelf.shelf_id == product.shelf_id:
                    shelf.current_inventory -= product.quantity
                    self.shelves[idx] = shelf
                    self.log_to_central(
                        "INFO", f"Inventory updated: {shelf.current_inventory} items of type {shelf.product} left on shelf {shelf.shelf_id}")

    def fleet_status_callback(self, msg):
        self.robots = msg.robot_status_list

    def shelf_query_callback(self, request, response):
        """
        Callback for the /database_query service.
        Returns shelf details based on the provided shelf_id.
        """
        shelf_found = False
        shelf_id = request.shelf_id

        for shelf in self.shelves:
            if shelf_id == shelf.shelf_id:
                response = shelf
                shelf_found = True
                self.log_to_central("INFO", f"Query successful for shelf_id: {shelf_id}")
                break

        if not shelf_found:
            default_pose = Pose()
            default_pose.position.x = 0.0
            default_pose.position.y = 0.0
            default_pose.position.z = 0.0
            default_pose.orientation.x = 0.0
            default_pose.orientation.y = 0.0
            default_pose.orientation.z = 0.0
            default_pose.orientation.w = 0.0
            response.shelf_location = default_pose
            response.shelf_capacity = 0
            response.current_inventory = 0
            self.log_to_central("WARN", f"Shelf_id {shelf_id} not found in database.")

        return response

    def robot_state_callback(self, request, response):
        """Returns the current state of a requested robot."""
        robot_found = False
        robot_id = request.robot_id

        for robot in self.robots:
            if robot_id == robot.robot_id:
                response = robot
                robot_found = True
                self.log_to_central("INFO", f"Query successful for robot_id: {robot_id}")
                break

        if not robot_found:
            default_pose = Pose()
            default_pose.position.x = 0.0
            default_pose.position.y = 0.0
            default_pose.position.z = 0.0
            default_pose.orientation.x = 0.0
            default_pose.orientation.y = 0.0
            default_pose.orientation.z = 0.0
            default_pose.orientation.w = 0.0

            response.current_location = default_pose
            response.battery_level = 0
            response.is_available = False
            self.log_to_central("WARN", f"Robot_id {robot_id} not found in database.")
        return response

    def inventory_update_callback(self, request, response):
        """
        Callback for the /update_inventory service.
        Updates the inventory of a shelf.
        """
        shelf_id = request.shelf_id
        new_inventory = request.new_inventory
        response.success = False

        for index, shelf in enumerate(self.shelves):
            if shelf_id == shelf.shelf_id:
                self.shelves[index].current_inventory = new_inventory
                response.success = True
                self.log_to_central("INFO", f"Inventory updated for shelf_id: {shelf_id}")
                break
        if not response.success:
            self.log_to_central("WARN", f"Shelf_id {shelf_id} not found in database.")

        return response

    def get_fleet_status_callback(self, request, response):

        if not self.robots:
            self.log_to_central("WARN", "NO Robot Data")
            return

        response.robot_status_list = self.robots

        self.log_to_central("INFO", 'Returning RobotFleetStatus.')
        return response

    def get_shelf_list_callback(self, request, response):

        if not self.shelves:
            self.log_to_central("WARN", "NO Shelf Data to return.")
            response.shelf_status_list = []
            return response

        response.shelf_status_list = self.shelves

        #self.log_to_central("INFO", "Returning ShelfList with {} shelves.".format(len(self.shelves)))
        return response

    def get_drop_off_pose_callback(self, request, response):
        response.pose = self.drop_off_pose
        return response

    def log_database(self):
        """
        Logs the current state of the database
        """
        self.database.update({"shelves": self.shelves})
        self.database.update({"robots": self.robots})
        self.log_to_central("INFO", "Current Database State:")
        #print(self.database)


def main(args=None):
    rclpy.init(args=args)
    shared_memory_module = SharedMemoryNode()
    rclpy.spin(shared_memory_module)
    shared_memory_module.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()