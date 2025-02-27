import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ShelfQuery, InventoryUpdate
from robot_interfaces.msg import RobotStatus
import json
import os
from ament_index_python.packages import get_package_share_directory


class DatabaseModule(Node):
    def __init__(self):
        super().__init__('database_module')

        database_dir = get_package_share_directory("task_management")
        database_name = "database"
        database_file = os.path.join(database_dir, "databases", database_name + ".json")

        # Initialize the database with sample data
        self.load_database_from_json(database_file)

        # Create services
        self.database_query_service = self.create_service(
            ShelfQuery, '/database_query', self.shelf_query_callback)
        self.inventory_update_service = self.create_service(
            InventoryUpdate, '/update_inventory', self.inventory_update_callback)

        # Create a publisher for robot status
        self.robot_status_publisher = self.create_publisher(
            RobotStatus, '/robot_status', 10)

        self.get_logger().info("Database Module is ready.")

    def load_database_from_json(self, database_json):
        with open(database_json, 'r') as file:
            data = json.load(file)

        self.database = data
        self.log_database()

    def shelf_query_callback(self, request, response):
        """
        Callback for the /database_query service.
        Returns shelf details based on the provided shelf_id.
        """
        shelf_found = False
        shelf_id = request.shelf_id

        for shelf in self.database.get("shelves"):
            if shelf_id == shelf.get("id"):
                response.shelf_location = shelf["location"]
                response.shelf_capacity = shelf["capacity"]
                response.current_inventory = shelf["inventory"]
                shelf_found = True
                self.get_logger().info(f"Query successful for shelf_id: {shelf_id}")
                break

        if not shelf_found:
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
        response.success = False

        for index, shelf in enumerate(self.database.get("shelves")):
            if shelf_id == shelf.get("id"):
                self.database["shelves"][index]["inventory"] = new_inventory
                response.success = True
                self.get_logger().info(f"Inventory updated for shelf_id: {shelf_id}")
                break
        if not response.success:
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
