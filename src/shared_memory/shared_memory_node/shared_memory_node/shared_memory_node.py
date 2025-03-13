import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ShelfQuery, InventoryUpdate, GetRobotStatus, GetRobotFleetStatus, GetShelfList
from robot_interfaces.msg import Task, RobotStatus, ShelfStatus, FleetStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import json
import os
import csv
from ament_index_python.packages import get_package_share_directory

# Custom JSON Encoder for handling ShelfStatus and Pose
class CustomJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        # Handle serialization for ShelfStatus
        if isinstance(obj, ShelfStatus):
            return {
                "shelf_id": obj.shelf_id,
                "shelf_location": obj.shelf_location,  # Pose is handled below
                "product": obj.product,
                "shelf_capacity": obj.shelf_capacity,
                "current_inventory": obj.current_inventory
            }
        # Handle serialization for Pose
        elif isinstance(obj, Pose):
            return {
                "position": {
                    "x": obj.position.x,
                    "y": obj.position.y,
                    "z": obj.position.z
                },
                "orientation": {
                    "x": obj.orientation.x,
                    "y": obj.orientation.y,
                    "z": obj.orientation.z,
                    "w": obj.orientation.w
                }
            }
        return super().default(obj)

class SharedMemoryNode(Node):
    def __init__(self):
        super().__init__('shared_memory_node')

        database_dir = get_package_share_directory("shared_memory_node")
        database_name = "shelves_database"
        database_file = os.path.join(database_dir, "databases", database_name + ".csv")

        # Initialize the database with sample data
        self.database = {"shelves": []}
        self.robots = []
        self.shelves = []

        self.load_inventory(database_file)
        self.database["shelves"] = self.shelves
        self.database["robots"] = self.robots
        self.tasks = []

        # Subscribers and Services (omitted for brevity)
        # Subscribers and services setup here...

        self.get_logger().info("Database Module is ready.")
        
        # New service for getting the inventory
        self.get_inventory_service = self.create_service(InventoryUpdate, 'get_inventory', self.get_inventory)
        
        
    def get_inventory(self, request, response):
        """
        ROS2 service handler to send inventory data.
        """
        inventory_data = []
        for shelf in self.shelves:
        # Create a ShelfStatus message for each shelf
            shelf_status = ShelfStatus()
            shelf_status.shelf_id = shelf.shelf_id
            shelf_status.shelf_location = shelf.shelf_location  # Assuming Pose is already in correct format
            shelf_status.product = shelf.product
            shelf_status.shelf_capacity = shelf.shelf_capacity
            shelf_status.current_inventory = shelf.current_inventory
        
            # Append the ShelfStatus message to the inventory data list
            inventory_data.append(shelf_status)

        # Set the response shelves field with the populated list
        response.shelves = inventory_data
        return response


    def load_inventory(self, database_file):
        """Reads inventory data from the CSV file and stores it in a dictionary."""
        try:
            with open(database_file, mode='r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    shelf = ShelfStatus()
                    shelf_pose = Pose()
                    shelf_pose.position.x = float(row["X"])
                    shelf_pose.position.y = float(row["Y"])
                    shelf_pose.position.z = float(row["Z"])
                    shelf_pose.orientation.x = float(row["Q_X"])
                    shelf_pose.orientation.y = float(row["Q_Y"])
                    shelf_pose.orientation.z = float(row["Q_Z"])
                    shelf_pose.orientation.w = float(row["Q_W"])

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

    def log_database(self):
        """
        Logs the current state of the database using the custom JSON encoder.
        """
        self.database.update({"shelves": self.shelves})
        self.database.update({"robots": self.robots})

        # Use the custom encoder to serialize the database to JSON
        self.get_logger().info("Current Database State:")
        self.get_logger().info(json.dumps(self.database, indent=2, cls=CustomJSONEncoder))

    # Other methods for task management, robot status, and inventory updates
    # The rest of your methods remain the same...

def main(args=None):
    rclpy.init(args=args)
    shared_memory_module = SharedMemoryNode()
    rclpy.spin(shared_memory_module)
    shared_memory_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

