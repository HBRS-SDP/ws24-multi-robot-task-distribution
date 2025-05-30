"""
Task Manager Node

This node is responsible for managing and assigning tasks to a fleet of robots in a warehouse simulation. 
It processes incoming orders, evaluates robot availability, and assigns tasks based on proximity, battery level, and availability. 
The node interacts with other services and topics to ensure efficient task distribution and execution.

### Responsibilities:
1. **Order Management**:
   - Subscribes to the `/order_requests` topic to receive new orders.
   - Processes orders by assigning tasks to the most suitable robot.
   - Publishes completed orders to the `/end_order` topic.

2. **Task Assignment**:
   - Allocates tasks to robots based on proximity to shelves, battery level, and availability.
   - Creates task messages for robots to execute, including moving to shelves and drop-off locations.

3. **Service Interactions**:
   - Calls the `/get_robot_fleet_status` service to get the status of all robots.
   - Calls the `/get_shelf_list` service to retrieve shelf details.
   - Calls the `/get_drop_off_pose` service to determine the drop-off location.
   - Calls the `/task_assignments` service to assign tasks to robots.

4. **Logging**:
   - Publishes logs to the `/central_logs` topic for centralized logging and monitoring.

### Key Features:
- Uses asynchronous programming with `asyncio` to handle service calls and task processing concurrently.
- Implements a multi-threaded executor to allow concurrent service calls and callbacks.
- Maintains an order queue to handle incoming orders sequentially.
- Dynamically calculates the best robot for a task based on a scoring system.

### Topics:
- **Subscribed**:
  - `/order_requests`: Receives new orders.
- **Published**:
  - `/end_order`: Publishes completed orders.
  - `/central_logs`: Publishes logs for monitoring.

### Services:
- **Clients**:
  - `/get_robot_fleet_status`: Retrieves the status of all robots.
  - `/get_shelf_list`: Retrieves the list of shelves and their details.
  - `/get_drop_off_pose`: Retrieves the drop-off location for completed tasks.
  - `/task_assignments`: Assigns tasks to robots.

### Methods:
- `log_to_central(level, message)`: Publishes logs to the central logging topic.
- `run_asyncio_loop()`: Runs the asyncio event loop in a separate thread.
- `create_task(coroutine)`: Schedules an asyncio task in the event loop.
- `order_callback(msg)`: Callback triggered when a new order is received.
- `process_orders()`: Continuously processes orders from the queue.
- `process_order(order)`: Processes a single order by calling necessary services and assigning tasks.
- `call_robot_fleet_service()`: Calls the `/get_robot_fleet_status` service.
- `call_shelf_list_service()`: Calls the `/get_shelf_list` service.
- `get_drop_off_pose()`: Calls the `/get_drop_off_pose` service.
- `allocate_task(robot_fleet_response, shelf_query_response, drop_off_pose, order)`: Allocates a task to the best robot.
- `get_drop_off_task(robot_id, drop_off_pose)`: Creates a task for the robot to move to the drop-off location.
- `call_task_assignment_service(task_list)`: Calls the `/task_assignments` service to assign tasks.
- `calculate_distance(location1, location2)`: Calculates the distance between two locations.

### Execution:
- The node is initialized and spun using a `MultiThreadedExecutor` to handle concurrent callbacks and service calls.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robot_interfaces.msg import Order, Task, Product, Logs
from robot_interfaces.srv import ShelfQuery, GetRobotStatus, GetRobotFleetStatus, TaskList, GetShelfList, GetPose
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

import asyncio
import math
from collections import deque
import threading

from sympy import false


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Callback group for services (to allow concurrent service calls)
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.order_subscriber = self.create_subscription(
            Order, '/order_requests', self.order_callback, 10)

        # Publishers
        self.order_end_publisher = self.create_publisher(Order, '/end_order', 10)
        self.log_publisher = self.create_publisher(Logs, '/central_logs', 10)

        # Service clients
        self.shelf_query_client = self.create_client(
            ShelfQuery, '/shelf_query', callback_group=self.callback_group)

        self.robot_fleet_client = self.create_client(
            GetRobotFleetStatus, '/get_robot_fleet_status', callback_group=self.callback_group)

        self.shelf_list_client = self.create_client(
            GetShelfList, '/get_shelf_list', callback_group=self.callback_group)

        self.task_assignment_client = self.create_client(
            TaskList, '/task_list', callback_group=self.callback_group)

        self.get_drop_off_pose_client = self.create_client(
            GetPose, '/get_drop_off_pose', callback_group=self.callback_group)

        # Robot status dictionary
        self.robots = {}  # Format: {robot_id: RobotStatus}

        # Order queue
        self.order_queue = deque()
        self.task_id_counter = 1

        # Start the order processing loop in a separate thread
        self.log_to_central("INFO", "Task Manager is ready.")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_asyncio_loop, daemon=True)
        self.thread.start()


    def log_to_central(self, level, message):
        """Publishes logs to the central logging topic."""
        log_msg = Logs()
        log_msg.timestamp =  self.get_clock().now().to_msg()
        log_msg.node_name = "Task Manager"
        log_msg.log_level = level
        log_msg.message = message
        self.log_publisher.publish(log_msg)


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
        self.log_to_central("INFO", f'Received order request: {msg}')
        self.order_queue.append(msg)
        self.create_task(self.process_orders())  # Trigger order processing

    async def process_orders(self):
        """
        Continuously processes orders from the queue.
        """
        self.log_to_central("INFO", "process_orders coroutine started.")  # Debug log
        while rclpy.ok():
            if self.order_queue:
                order = self.order_queue.popleft()
                self.log_to_central("INFO", f'Processing order: {order}')
                success = await self.process_order(order)
                if not success:
                    await asyncio.sleep(1)
            else:
                await asyncio.sleep(1)  # Sleep if the queue is empty

    async def process_order(self, order):
        """
        Asynchronously process the order by calling the two services concurrently.
        """

        try:
            # Call the two services concurrently using asyncio.gather
            robot_fleet_response, shelf_query_response, drop_off_pose = await asyncio.gather(
                self.call_robot_fleet_service(),
                self.call_shelf_list_service(),
                self.get_drop_off_pose()
            )

            task_assigned = self.allocate_task(robot_fleet_response, shelf_query_response, drop_off_pose, order)

            if task_assigned:
                success = await self.call_task_assignment_service(task_assigned)
                if success:
                    self.order_end_publisher.publish(order)
                    return success

                # If task assignment failed, push the order back to the queue
                self.log_to_central("WARN", "Task assignment failed. Re-queueing order.")
                self.order_queue.append(order)
                return False
            else:
                # If no robot was available, push the order back to the queue
                self.log_to_central("WARN", "No available robots to assign task. Re-queueing order.")
                self.order_queue.append(order)
                return False

        except Exception as e:
            self.log_to_central("ERROR", f'Error calling services: {e}')
            # Re-queue the order if an error occurs
            self.order_queue.append(order)
            return False

    async def call_robot_fleet_service(self):
        """
        Call the /get_robot_fleet_status service asynchronously.
        """
        while not self.robot_fleet_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /get_robot_fleet_status service...')

        request = GetRobotFleetStatus.Request()
        future = self.robot_fleet_client.call_async(request)
        await future
        return future.result()

    async def call_shelf_list_service(self):
        """
        Call the /shelf_query service asynchronously.
        """
        while not self.shelf_list_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /shelf_query service...')

        # returns a list of shelf details
        request = GetShelfList.Request()
        future = self.shelf_list_client.call_async(request)
        await future
        return future.result()

    async def get_drop_off_pose(self):
        """
        Get the drop_off_pose from the shared_memory_node.
        """
        while not self.get_drop_off_pose_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for get_drop_off_pose service...')

        request = GetPose.Request()
        future = self.get_drop_off_pose_client.call_async(request)

        await future
        return future.result()

    def allocate_task(self, robot_fleet_response, shelf_query_response, drop_off_pose, order):
        # Sort the order's product list based on shelf_id
        order.product_list.sort(key=lambda product: product.shelf_id)
        """
        Allocates a task to the best robot based on proximity, battery level, and availability.
        Returns the Task message if a robot was assigned, otherwise None.
        """
        best_robot_id = None
        best_score = -1

        # A list of RobotStatus
        robot_fleet_list = robot_fleet_response.robot_status_list
        shelf_details = shelf_query_response.shelf_status_list

        for robot in robot_fleet_list:
            if robot.is_available:
                # Calculate proximity score (distance to shelf)
                robot_location = robot.current_location

                first_shelf = None
                for shelf in shelf_details:
                    if shelf.shelf_id == order.product_list[0].shelf_id:
                        first_shelf = shelf
                        break

                if not first_shelf:
                    self.log_to_central("WARN", f"Shelf {order.product_list[0].shelf_id} not found.")
                    return None

                shelf_location = first_shelf.shelf_location
                distance = self.calculate_distance(robot_location, shelf_location)

                # Calculate battery score (higher battery is better)
                battery_score = robot.battery_level

                # Combined score (lower distance and higher battery are better)
                score = (1 / (distance + 1)) * battery_score

                if score > best_score:
                    best_score = score
                    best_robot_id = robot.robot_id

        if best_robot_id:
            # Create a Task message for each product in the order
            task_list = []
            for product in order.product_list:
                shelf = next((s for s in shelf_details if s.shelf_id == product.shelf_id), None)
                task_msg = Task()
                task_msg.task_id = self.task_id_counter
                task_msg.robot_id = best_robot_id
                task_msg.shelf_id = product.shelf_id
                task_msg.shelf_location = shelf.shelf_location
                task_msg.item = shelf.product
                task_msg.item_amount = product.quantity
                task_msg.task_type = f"Move to Shelf {product.shelf_id}"
                self.task_id_counter += 1
                task_list.append(task_msg)

            drop_off_task = self.get_drop_off_task(best_robot_id, drop_off_pose)
            task_list.append(drop_off_task)
            self.log_to_central("INFO", f"Assigned Order to robot {best_robot_id}: {task_list}")

            return task_list
        else:
            self.log_to_central("WARN", "No available robots to assign task.")
            return None

    def get_drop_off_task(self, robot_id, drop_off_pose):
        # Calculate specific drop-off location for the robot
        drop_off_x_offset = 0.6  # 60 cm offset
        specific_drop_off_pose = Pose()
        specific_drop_off_pose.position.x = drop_off_pose.pose.position.x + robot_id * drop_off_x_offset
        specific_drop_off_pose.position.y = drop_off_pose.pose.position.y
        specific_drop_off_pose.position.z = drop_off_pose.pose.position.z
        specific_drop_off_pose.orientation = drop_off_pose.pose.orientation

        # Add a final task to move to the specific drop-off location
        drop_off_task = Task()
        drop_off_task.task_id = self.task_id_counter
        drop_off_task.robot_id = robot_id
        drop_off_task.shelf_id = 0
        drop_off_task.shelf_location = specific_drop_off_pose
        drop_off_task.item = ""
        drop_off_task.item_amount = 0
        drop_off_task.task_type = "Move to Drop-off"
        self.task_id_counter += 1
        return drop_off_task

    async def call_task_assignment_service(self, task_list):
        """
        Call the /task_assignments service to assign the task to the robot.
        Returns True if the task was successfully assigned, False otherwise.
        """
        while not self.task_assignment_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /task_list service...')

        request = TaskList.Request()
        request.task_list = task_list
        future = self.task_assignment_client.call_async(request)
        await future
        return future.result().success

    def calculate_distance(self, location1: Pose, location2: Pose):
        """
        Calculates the distance between two locations.
        """
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + (location2.position.y - location1.position.y) ** 2)


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