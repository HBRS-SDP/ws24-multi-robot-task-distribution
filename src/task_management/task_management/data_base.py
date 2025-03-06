import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ShelfQuery, InventoryUpdate, RobotStatus
from robot_interfaces.msg import Task
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import json
import os
import csv
from ament_index_python.packages import get_package_share_directory


class DatabaseModule(Node):
    def __init__(self):
        super().__init__('database_module')

        database_dir = get_package_share_directory("task_management")
        database_name = "shelves_database"
        database_file = os.path.join(database_dir, "databases", database_name + ".csv")

        # Initialize the database with sample data
        self.database = {"shelves": []}
        self.database["robots"] = [
            {
                "id": 1,
                "location": [2.0, 3.0, 0.0],
                "battery_level": 100.0,
                "is_available": True,
                "status": "idle"
            },
            {
                "id": 2,
                "location": [4.0, 3.0, 0.0],
                "battery_level": 95.0,
                "is_available": True,
                "status": "idle"
            }]

        self.shelves = self.database.get("shelves")
        self.robots = self.database.get("robots")

        self.tasks = []

        self.load_inventory_data(database_file)

        # Older database using json
        # self.load_database_from_json(database_file)

        # Subscribers
        self.task_start_sub = self.create_subscription(Task, '/start_task', self.task_start_callback, 10)
        self.task_end_sub = self.create_subscription(Int32, '/end_task', self.task_end_callback, 10)

        # Services
        self.database_query_service = self.create_service(
            ShelfQuery, '/database_query', self.shelf_query_callback)
        self.inventory_update_service = self.create_service(
            InventoryUpdate, '/update_inventory', self.inventory_update_callback)
        self.robot_state_service = self.create_service(
            RobotStatus, '/get_robot_state', self.robot_state_callback)

        self.get_logger().info("Database Module is ready.")

    def load_inventory_data(self, database_file):
        """Reads inventory data from the CSV file and stores it in a dictionary."""

        try:
            with open(database_file, mode='r') as file:
                reader = csv.DictReader(file)
                self.database["shelves"] = [
                    {
                        "id": int(row["id"]),
                        "location": [float(row["location_x"]), float(row["location_y"]), float(row["location_z"])],
                        "product": row["product"],
                        "capacity": int(row["capacity"]),
                        "inventory": int(row["inventory"])
                    }
                    for row in reader
                ]

            self.get_logger().info("Inventory successfully loaded from CSV.")
            self.log_database()
        except Exception as e:
            self.get_logger().error(f"Error loading inventory from CSV: {e}")

    def load_database_from_json(self, database_json):
        with open(database_json, 'r') as file:
            data = json.load(file)

        self.database = data
        self.log_database()

    def task_start_callback(self, msg):
        """Updates robot status and task history when a task starts."""
        self.get_logger().info(f"Robot {msg.robot_id} started task at shelf {msg.shelf_id} for task id {msg.task_id}")
        task = {'task_id': msg.task_id, 'robot_id': msg.robot_id, 'shelf_id': msg.shelf_id, 'item': msg.item, 'amount': msg.item_amount,
                'type': msg.task_type}
        self.tasks.append(task)
        self.update_robot_status(msg.robot_id, msg.task_type, False)

    def task_end_callback(self, msg):
        """Updates robot status and inventories when it reaches a shelf."""

        task = next((task for task in self.tasks if task.get("id") == msg.data), None)
        robot_id = task.get('robot_id')
        self.get_logger().info(f"Robot {robot_id} has finished the task {msg.data}")

        self.update_inventory_status(task)
        self.update_robot_status(robot_id = robot_id, new_status = 'idle', availability = True)


    def update_robot_status(self, robot_id: int, new_status: str, availability: bool):
        if not self.robots:
            return

        for idx, robot in enumerate(self.robots):
            if robot.get("id") == robot_id:
                robot["status"] = new_status
                robot["is_available"] = availability
                self.robots[idx] = robot
                break

    def update_inventory_status(self, task):
        for idx, shelf in self.shelves:
            if shelf.get('id') == task.get('shelf_id'):
                shelf['inventory'] -= task.get('amount')
                self.shelves[idx] = shelf
                self.get_logger().info(
                    f"📉 Inventory updated: {shelf.get('inventory')} items of type {shelf.get('product')} left on shelf {shelf.get('id')}")



    def shelf_query_callback(self, request, response):
        """
        Callback for the /database_query service.
        Returns shelf details based on the provided shelf_id.
        """
        shelf_found = False
        shelf_id = request.shelf_id

        for shelf in self.database.get("shelves"):
            if shelf_id == shelf.get("id"):
                location_point = Point()
                location = shelf.get("location")
                location_point.x = location[0]
                location_point.y = location[1]
                location_point.z = location[2]
                response.shelf_location = location_point
                response.shelf_capacity = shelf["capacity"]
                response.current_inventory = shelf["inventory"]
                shelf_found = True
                self.get_logger().info(f"Query successful for shelf_id: {shelf_id}")
                break

        if not shelf_found:
            response.shelf_location = Point(0, 0, 0)
            response.shelf_capacity = 0
            response.current_inventory = 0
            self.get_logger().warn(f"Shelf_id {shelf_id} not found in database.")
        return response

    def robot_state_callback(self, request, response):
        """Returns the current state of a requested robot."""
        robot_found = False
        robot_id = request.robot_id

        for robot in self.database.get("robots"):
            if robot_id == robot.get("id"):
                location_point = Point()
                location = robot.get("location")
                location_point.x = location[0]
                location_point.y = location[1]
                location_point.z = location[2]
                response.current_location = location_point
                response.battery_level = robot["battery_level"]
                response.is_available = robot["is_available"]
                robot_found = True
                self.get_logger().info(f"Query successful for shelf_id: {robot_id}")
                break

        if not robot_found:
            response.current_location = Point(0, 0, 0)
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
