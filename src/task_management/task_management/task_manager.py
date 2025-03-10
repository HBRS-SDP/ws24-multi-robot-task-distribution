import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order, Task
from robot_interfaces.srv import ShelfQuery, GetRobotStatus, GetRobotFleetStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
import asyncio
import math

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Callback group for services (to allow concurrent service calls)
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.order_subscriber = self.create_subscription(
            Order, '/order_requests', self.order_callback, 10)
        


        # Publishers
        self.task_publisher = self.create_publisher(Task, '/task_assignments', 10)

        # Service clients
        self.shelf_query_client = self.create_client(
            ShelfQuery, '/shelf_query', callback_group=self.callback_group)
        
        self.robot_fleet_client = self.create_client(
            GetRobotFleetStatus, '/get_robot_fleet_status', callback_group=self.callback_group)
        


        # Robot status dictionary
        self.robots = {}  # Format: {robot_id: RobotStatus}
        self.task_id_counter = 1

        self.get_logger().info("Task Manager is ready.")

    def order_callback(self, msg):
        """
        Callback triggered when a new order is received.
        This function schedules the asynchronous task in a new event loop.
        """
        self.get_logger().info(f'Received order request: {msg}')

        # Create a new event loop for the asynchronous task
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Run the asynchronous task in the new event loop
        loop.run_until_complete(self.process_order(msg))
        loop.close()

    async def process_order(self, msg):
        """
        Asynchronously process the order by calling the two services concurrently.
        """
        self.get_logger().info('Calling services concurrently...')

        try:
            # Call the two services concurrently using asyncio.gather
            robot_fleet_response, database_query_response = await asyncio.gather(
                self.call_robot_fleet_service(),
                self.call_database_query_service()
            )

            # Use the responses to allocate tasks to a robot
            self.allocate_task(robot_fleet_response, database_query_response)

        except Exception as e:
            self.get_logger().error(f'Error calling services: {e}')

    async def call_robot_fleet_service(self):
        """
        Call the /get_robot_fleet_status service asynchronously.
        """
        while not self.robot_fleet_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /get_robot_fleet_status service...')

        request = GetRobotFleetStatus.Request()
        future = self.robot_fleet_client.call_async(request)
        await future
        return future.result()

    async def call_database_query_service(self):
        """
        Call the /shelf_query service asynchronously.
        """
        while not self.shelf_query_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /shelf_query service...')

        request = ShelfQuery.Request()
        request.shelf_id = 1
        future = self.shelf_query_client.call_async(request)
        await future
        return future.result()

    def allocate_task(self, robot_fleet_response, database_query_response):
        """
        Allocates a task to the best robot based on proximity, battery level, and availability.
        """
        best_robot_id = None
        best_score = -1

        # A list of RobotStaus    
        robot_fleet_list = robot_fleet_response.robot_status_list
        shelf_details = database_query_response


        for robot in robot_fleet_list:
            if robot.is_available:

                # Calculate proximity score (distance to shelf)
                robot_location = robot.current_location
                shelf_location = shelf_details.shelf_location
                distance = self.calculate_distance(robot_location, shelf_location)

                # Calculate battery score (higher battery is better)
                battery_score = robot.battery_level

                # Combined score (lower distance and higher battery are better)
                score = (1 / (distance + 1)) * battery_score

                if score > best_score:
                    best_score = score
                    best_robot_id = robot.robot_id

        if best_robot_id:
            # Assign task to the best robot
            # TODO: hard coded for testing
            task_msg = Task()
            task_msg.task_id = self.task_id_counter
            task_msg.robot_id = best_robot_id
            task_msg.shelf_id = 1  
            task_msg.shelf_location = shelf_details.shelf_location
            task_msg.item = "A"
            task_msg.item_amount = 3
            task_msg.task_type = "pickup"
            self.task_publisher.publish(task_msg)
            self.task_id_counter += 1
            self.get_logger().info(f"Assigned task to robot {best_robot_id}: {task_msg}")
        else:
            self.get_logger().warn("No available robots to assign task.")



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
