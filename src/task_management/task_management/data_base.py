import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from task_management.srv import Database, InventoryUpdate
from task_management.msg import RobotStatus
import json

class DatabaseModule(Node):
    def __init__(self):
        super().__init__('database_module')

        # Initialize the database with sample data
        self.database = {
            "shelves": {
                1: {"location": "A1", "capacity": 10, "inventory": 5},
                2: {"location": "B2", "capacity": 15, "inventory": 10},
                3: {"location": "C3", "capacity": 20, "inventory": 15},
            },
            "robots": {
                "robot_1": {"location": "Station", "battery_level": 100.0, "is_available": True},
                "robot_2": {"location": "Station", "battery_level": 95.0, "is_available": True},
            },
            "order_history": [],
        }

        # Create services
        self.database_query_service = self.create_service(
            Database, '/database_query', self.database_query_callback)
        self.inventory_update_service = self.create_service(
            InventoryUpdate, '/update_inventory', self.inventory_update_callback)

        # Create a publisher for robot status
        self.robot_status_publisher = self.create_publisher(
            RobotStatus, '/robot_status', 10)

        self.get_logger().info("Database Module is ready.")

    def database_query_callback(self, request, response):
        """
        Callback for the /database_query service.
        Returns shelf details based on the provided shelf_id.
        """
        shelf_id = request.shelf_id
        if shelf_id in self.database["shelves"]:
            shelf = self.database["shelves"][shelf_id]
            response.shelf_location = shelf["location"]
            response.shelf_capacity = shelf["capacity"]
            response.current_inventory = shelf["inventory"]
            self.get_logger().info(f"Query successful for shelf_id: {shelf_id}")
        else:
            response.shelf_location = ""
            response.shelf_capacity = 0
            response.current_inventory = 0
            self.get_logger().warn(f"Shelf_id {shelf_id} not found in database.")
        return response

    def inventory_update_callback(self, request, response):
        """
        Callback for the /update_inventory service.
        Updates the inventory of a shelf.
        """
        shelf_id = request.shelf_id
        new_inventory = request.new_inventory

        if shelf_id in self.database["shelves"]:
            self.database["shelves"][shelf_id]["inventory"] = new_inventory
            response.success = True
            self.get_logger().info(f"Inventory updated for shelf_id: {shelf_id}")
        else:
            response.success = False
            self.get_logger().warn(f"Shelf_id {shelf_id} not found in database.")
        return response

    def log_database(self):
        """
        Logs the current state of the database
        """
        self.get_logger().info("Current Database State:")
        self.get_logger().info(json.dumps(self.database, indent=2))

def main(args=None):
    rclpy.init(args=args)
    database_module = DatabaseModule()
    rclpy.spin(database_module)
    database_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()