import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from robot_interfaces.srv import ShelfQuery, InventoryUpdate, GetRobotStatus, GetRobotFleetStatus, GetShelfList, GetPose
from robot_interfaces.msg import ShelfStatus, FleetStatus, Order
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import json
import os
import csv


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
        
        # Publishers
        self.log_publisher = self.create_publisher(String, '/central_logs', 10)

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

        self.get_logger().info("Database Module is ready.")

    def log_to_central(self, level, message, robot_namespace=None, log_source="SharedMemory"):
        """Publishes logs to the central logging topic."""
        log_msg = String()
        log_msg.data = f"SharedMemory|{level}|{message}"
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
                        self.get_logger().info(f"Drop off location loaded with id: {row['id']}")
                    else:
                        # "product" in row and "capacity" in row and "inventory" in row:
                        shelf = ShelfStatus()
                        shelf.shelf_id = int(row["id"])
                        shelf.shelf_location = shelf_pose
                        shelf.product = row["product"]
                        shelf.shelf_capacity = int(row["capacity"])
                        shelf.current_inventory = int(row["inventory"])
                        self.shelves.append(shelf)

            self.get_logger().info("Inventory successfully loaded from CSV.")
            self.log_database()
        except Exception as e:
            self.get_logger().error(f"Error loading inventory from CSV: {e}")



    def order_end_callback(self, order):
        """Updates robot status and inventories when it reaches a shelf."""

        if not order.product_list:
            self.get_logger().warn("No product list in order.")
            return

        self.update_inventory_status(order)

        self.log_database()


    def update_inventory_status(self, order):
        if not self.shelves:
            self.get_logger().info("No shelves available to update inventory.")
            return

        for product in order.product_list:
            for idx, shelf in enumerate(self.shelves):
                if shelf.shelf_id == product.shelf_id:
                    shelf.current_inventory -= product.quantity
                    self.shelves[idx] = shelf
                    self.get_logger().info(
                        f"📉 Inventory updated: {shelf.current_inventory} items of type {shelf.product} left on shelf {shelf.shelf_id}")

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
                self.get_logger().info(f"Query successful for shelf_id: {shelf_id}")
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
            self.get_logger().warn(f"Shelf_id {shelf_id} not found in database.")

        return response

    def robot_state_callback(self, request, response):
        """Returns the current state of a requested robot."""
        robot_found = False
        robot_id = request.robot_id

        for robot in self.robots:
            if robot_id == robot.robot_id:
                response = robot
                robot_found = True
                self.get_logger().info(f"Query successful for robot_id: {robot_id}")
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
            self.get_logger().warn(f"Robot_id {robot_id} not found in database.")
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
                self.get_logger().info(f"Inventory updated for shelf_id: {shelf_id}")
                break
        if not response.success:
            self.get_logger().warn(f"Shelf_id {shelf_id} not found in database.")

        return response

    def get_fleet_status_callback(self, request, response):

        if not self.robots:
            print("NO Robot Data")
            return

        response.robot_status_list = self.robots

        self.get_logger().info('Returning RobotFleetStatus.')
        return response

    def get_shelf_list_callback(self, request, response):

        if not self.shelves:
            self.get_logger().warn("NO Shelf Data to return.")
            response.shelf_status_list = []
            return response

        response.shelf_status_list = self.shelves

        self.get_logger().info("Returning ShelfList with {} shelves.".format(len(self.shelves)))
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
        self.get_logger().info("Current Database State:")
        #print(self.database)


def main(args=None):
    rclpy.init(args=args)
    shared_memory_module = SharedMemoryNode()
    rclpy.spin(shared_memory_module)
    shared_memory_module.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
