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


        # Older database using json
        # self.load_database_from_json(database_file)

        # Subscribers
        self.task_start_sub = self.create_subscription(Task, '/start_task', self.task_start_callback, 10)
        self.task_end_sub = self.create_subscription(Int32, '/end_task', self.task_end_callback, 10)
        self.fleet_status_sub = self.create_subscription(FleetStatus, '/fleet_status', self.fleet_status_callback, 10)

        # Services
        self.database_query_service = self.create_service(
            ShelfQuery, '/shelf_query', self.shelf_query_callback)
        self.inventory_update_service = self.create_service(
            InventoryUpdate, '/update_inventory', self.inventory_update_callback)
        self.robot_state_service = self.create_service(
            GetRobotStatus, '/get_robot_state', self.robot_state_callback)
        
        self.robot_fleet_service = self.create_service(GetRobotFleetStatus, '/get_robot_fleet_status', self.get_fleet_status_callback)
        self.shelf_list_service = self.create_service(GetShelfList, '/get_shelf_list', self.get_shelf_list_callback)

        self.get_logger().info("Database Module is ready.")


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


    def load_database_from_json(self, database_json):
        with open(database_json, 'r') as file:
            data = json.load(file)

        self.database = data
        self.log_database()


    def task_start_callback(self, msg):
        """Updates robot status and task history when a task starts."""
        self.get_logger().info(f"Robot {msg.robot_id} started task {msg.task_type} at shelf {msg.shelf_id} for task id {msg.task_id}")
        task = {'task_id': msg.task_id, 'robot_id': msg.robot_id, 'shelf_id': msg.shelf_id, 'item': msg.item, 'amount': msg.item_amount,
                'type': msg.task_type}
        self.tasks.append(task)
        print(self.tasks)
        self.update_robot_status(msg.robot_id, msg.task_type, False)
        self.log_database()


    def task_end_callback(self, msg):
        """Updates robot status and inventories when it reaches a shelf."""

        task = next((task for task in self.tasks if task.get("task_id") == msg.data), None)

        if not task:
            self.get_logger().info(f"The task with id {msg.data} was not found.")
            return

        robot_id = task.get('robot_id')
        self.update_inventory_status(task)
        self.update_robot_status(robot_id = robot_id, new_status = 'idle', availability = True)
        self.tasks.remove(task)

        self.get_logger().info(f"Robot {robot_id} has finished the task {msg.data}")
        self.log_database()


    def fleet_status_callback(self, msg):
        self.robots = msg.robot_status_list
        self.database["robots"] =  self.robots


    def update_robot_status(self, robot_id: int, new_status: str, availability: bool):
        if not self.robots:
            return

        for idx, robot in enumerate(self.robots):
            if robot.robot_id == robot_id:
                robot.status = new_status
                robot.is_available = availability
                self.robots[idx] = robot
                break


    def update_inventory_status(self, task):
        if not self.shelves:
            print("NO SHELF DATA")
            return

        for idx, shelf in enumerate(self.shelves):
            if shelf.shelf_id == task.get('shelf_id'):
                shelf.current_inventory -= task.get('amount')
                self.shelves[idx] = shelf
                self.get_logger().info(
                    f"📉 Inventory updated: {shelf.current_inventory} items of type {shelf.product} left on shelf {shelf.shelf_id}")


    def shelf_query_callback(self, request, response):
        """
        Callback for the /database_query service.
        Returns shelf details based on the provided shelf_id.
        """
        shelf_found = False
        shelf_id = request.shelf_id

        for shelf in self.shelves:
            if shelf_id == shelf.shelf_id:
                response.shelf_location = shelf.shelf_location
                response.shelf_capacity = shelf.shelf_capacity
                response.current_inventory = shelf.current_inventory
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
            print("NO Shelf Data")
            return

        response.shelf_status_list = self.shelves

        self.get_logger().info('Returning ShelfList.')
        return response


    def log_database(self):
        """
        Logs the current state of the database
        """
        self.database.update({"shelves": self.shelves})
        self.database.update({"robots": self.robots})
        self.get_logger().info("Current Database State:")
        #self.get_logger().info(self.database)
        print(self.database)


def main(args=None):
    rclpy.init(args=args)
    shared_memory_module = SharedMemoryNode()
    rclpy.spin(shared_memory_module)
    shared_memory_module.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
