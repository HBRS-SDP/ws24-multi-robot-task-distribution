import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order, Task
from robot_interfaces.srv import ShelfQuery, RobotStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
import math

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Callback group for services (to allow concurrent service calls)
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.order_subscriber = self.create_subscription(
            Order, '/order_requests', self.order_callback, 10)
        
        self.robot_status_subscriber = self.create_subscription(
            RobotStatus, '/robot_status', self.robot_status_callback, 10)

        # Publishers
        self.task_publisher = self.create_publisher(Task, '/task_assignments', 10)

        # Service clients
        self.database_client = self.create_client(
            ShelfQuery, '/database_query', callback_group=self.callback_group)

        # Robot status dictionary
        self.robots = {}  # Format: {robot_id: RobotStatus}

        
        
        # test
        self.robots = [
            {
            "id": 1,
            "location": Point(2,0,3),
            "battery_level": 100.0,
            "is_available": True
            },
            {
            "id": 2,
            "location": Point(4,0,3),
            "battery_level": 95.0,
            "is_available": True
            }]

        # Wait for the database service to be available
        while not self.database_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Database service not available, waiting...')

        self.get_logger().info("Task Manager is ready.")

    def order_callback(self, msg):
        """
        Callback for the /order_requests topic.
        Processes order requests and allocates tasks to robots.
        """
        self.get_logger().info(f"Received order request: {msg}")

        # Query the database for shelf details
        shelf_id = msg.shelf_id
        database_request = ShelfQuery()
        database_request.shelf_id = shelf_id
        future = self.database_client.call_async(database_request)
        future.add_done_callback(lambda future: self.database_query_callback(future, msg))

    def database_query_callback(self, future, order_msg):
        """
        Callback for the database query response.
        Allocates tasks to robots based on shelf details.
        """
        try:
            response = future.result()
            if response.shelf_location:
                self.get_logger().info(f"Shelf details: {response}")
                # Allocate task to the best robot
                self.allocate_task(order_msg, response)
            else:
                self.get_logger().warn(f"Shelf ID {order_msg.shelf_id} not found in database.")
        except Exception as e:
            self.get_logger().error(f"Database query failed: {e}")

    def allocate_task(self, order_msg, shelf_details):
        """
        Allocates a task to the best robot based on proximity, battery level, and availability.
        """
        best_robot_id = None
        best_score = -1


        #for robot_id, robot_status in self.robots.items():
            #if robot_status.is_available:
        for robot in self.robots:
            if robot.get("is_available"):

                # Calculate proximity score (distance to shelf)
                robot_location = robot.get("location")
                shelf_location = shelf_details.shelf_location
                distance = self.calculate_distance(robot_location, shelf_location)

                # Calculate battery score (higher battery is better)
                battery_score = robot.get("battery_level")

                # Combined score (lower distance and higher battery are better)
                score = (1 / (distance + 1)) * battery_score

                if score > best_score:
                    best_score = score
                    best_robot_id = robot.get("id")

        if best_robot_id:
            # Assign task to the best robot
            task_msg = Task()
            task_msg.robot_id = best_robot_id
            task_msg.shelf_id = order_msg.shelf_id
            task_msg.shelf_location = shelf_details.shelf_location
            task_msg.task_type = "pickup"  # Example task type
            self.task_publisher.publish(task_msg)
            self.get_logger().info(f"Assigned task to robot {best_robot_id}: {task_msg}")
        else:
            self.get_logger().warn("No available robots to assign task.")

    def robot_status_callback(self, msg: RobotStatus):
        """
        Callback for the /robot_status topic.
        Updates the status of robots in the fleet.
        """
        for index, robot in enumerate(self.robots):
            if robot.get("id") == msg.id:
                new_status =   {
                            "id": msg.id,
                            "location": msg.location,
                            "battery_level": msg.battery_level,
                            "is_available": msg.is_available
                            }
                self.robots[index] = new_status

    def calculate_distance(self, location1: Point, location2: Point):
        """
        Calculates the distance between two locations.
        """
        return math.sqrt((location2.x - location1.x) ** 2 + (location2.z - location1.z) ** 2)

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()

    # Use a MultiThreadedExecutor to handle service callbacks concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(task_manager, executor=executor)

    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
