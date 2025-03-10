import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robot_interfaces.msg import Order, Task
from robot_interfaces.srv import ShelfQuery, GetRobotStatus, GetRobotFleetStatus, TaskList
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
import asyncio
import math
from collections import deque
import threading

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
        
        self.task_assignment_client = self.create_client(
            TaskList, '/task_assignments', callback_group=self.callback_group)

        # Robot status dictionary
        self.robots = {}  # Format: {robot_id: RobotStatus}

        # Order queue
        self.order_queue = deque()
        self.task_id_counter = 1

        # Start the order processing loop in a separate thread
        self.get_logger().info("Task Manager is ready.")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_asyncio_loop, daemon=True)
        self.thread.start()

    def run_asyncio_loop(self):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def create_task(self, coroutine):
        """Schedule an asyncio task in the event loop."""
        asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def order_callback(self, msg):
        """
        Callback triggered when a new order is received.
        Adds the order to the queue.
        """
        self.get_logger().info(f'Received order request: {msg}')
        self.order_queue.append(msg)
        self.create_task(self.process_orders())  # Trigger order processing

    async def process_orders(self):
        """
        Continuously processes orders from the queue.
        """
        self.get_logger().info("process_orders coroutine started.")  # Debug log
        while rclpy.ok():
            if self.order_queue:
                order = self.order_queue.popleft()
                self.get_logger().info(f'Processing order: {order}')
                await self.process_order(order)
            else:
                self.get_logger().info(f'No orders.')
                await asyncio.sleep(1)  # Sleep if the queue is empty

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
            task_assigned = self.allocate_task(robot_fleet_response, database_query_response)

            if task_assigned:
                # Call the /task_assignments service to assign the task to the robot
                success = await self.call_task_assignment_service(task_assigned)
                if not success:
                    # If task assignment failed, push the order back to the queue
                    self.get_logger().warn("Task assignment failed. Re-queueing order.")
                    self.order_queue.append(msg)
            else:
                # If no robot was available, push the order back to the queue
                self.get_logger().warn("No available robots to assign task. Re-queueing order.")
                self.order_queue.append(msg)

        except Exception as e:
            self.get_logger().error(f'Error calling services: {e}')
            # Re-queue the order if an error occurs
            self.order_queue.append(msg)

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
        Returns the Task message if a robot was assigned, otherwise None.
        """
        best_robot_id = None
        best_score = -1

        print(database_query_response)

        # A list of RobotStatus    
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
            # Create a Task message
            task_msg = Task()
            task_msg.task_id = self.task_id_counter
            task_msg.robot_id = best_robot_id
            task_msg.shelf_id = 1  
            task_msg.shelf_location = shelf_details.shelf_location
            task_msg.item = "A"
            task_msg.item_amount = 3
            task_msg.task_type = "pickup"
            self.task_id_counter += 1
            self.get_logger().info(f"Assigned task to robot {best_robot_id}: {task_msg}")
            return task_msg
        else:
            self.get_logger().warn("No available robots to assign task.")
            return None

    async def call_task_assignment_service(self, task):
        """
        Call the /task_assignments service to assign the task to the robot.
        Returns True if the task was successfully assigned, False otherwise.
        """
        while not self.task_assignment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /task_assignments service...')

        request = TaskList.Request()
        request.task_list = [task]
        future = self.task_assignment_client.call_async(request)
        await future
        return future.result().success

    def calculate_distance(self, location1: Pose, location2: Pose):
        """
        Calculates the distance between two locations.
        """
        return math.sqrt((location2.position.x - location1.position.x) ** 2 + (location2.position.y - location1.position.y) ** 2)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    task_manager = TaskManager()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    try:
        # Spin the node
        rclpy.spin(task_manager, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()