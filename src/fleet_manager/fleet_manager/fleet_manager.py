import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from robot_interfaces.msg import RobotStatus, Task, FleetStatus
from robot_interfaces.srv import TaskList
import uuid
from std_msgs.msg import String

class FleetManager(Node):
    def __init__(self, num_robots):
        super().__init__('fleet_manager')
       
        self.log_publisher = self.create_publisher(String, '/central_logs', 10)

        self.robots = {}
        self.goal_handles = {}
        self.previous_feedback = {}
        self.robot_to_goal_id = {}

        # Initialize robots and subscriptions
        for i in range(1, num_robots + 1):
            robot_name = f'robot_{i}'
            odom_topic = f'/{robot_name}/odom'
            self.robots[robot_name] = {
                "id": i,
                "location": Pose(),
                "battery_level": 100.0,
                "is_available": True,
                "status": "idle"
            }
            self.create_subscription(Odometry, odom_topic, lambda msg, name=robot_name: self.odom_callback(msg, name), 10)

        # Create a QoS profile for fleet status
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create a publisher for fleet status with the QoS profile
        self.fleet_status_publisher = self.create_publisher(FleetStatus, 'fleet_status', qos_profile)

        # Create a timer to periodically publish fleet status
        self.timer = self.create_timer(1.0, self.publish_fleet_status)

        # Create a service for task list
        self.srv = self.create_service(TaskList, 'task_list', self.handle_task_list)
     
    def log_to_central(self, level, message, robot_namespace=None, log_source="FleetManager"):
        """Publishes logs to the central logging topic."""
        log_msg = String()
        log_msg.data = f"FleetManager|{level}|{message}"
        self.log_publisher.publish(log_msg)
        
    def odom_callback(self, msg, robot_name):
      if robot_name in self.robots:
        self.robots[robot_name]["location"] = msg.pose.pose

    def publish_fleet_status(self):
        fleet_status_msg = FleetStatus()
        fleet_status_msg.robot_status_list = []
        for robot in self.robots.values():
            robot_status = RobotStatus()
            robot_status.robot_id = robot["id"]
            robot_status.current_location = robot["location"]
            robot_status.battery_level = robot["battery_level"]
            robot_status.is_available = robot["is_available"]
            robot_status.status = robot["status"]
            fleet_status_msg.robot_status_list.append(robot_status)
        self.fleet_status_publisher.publish(fleet_status_msg)

    def handle_task_list(self, request, response):
        try:
            if request.task_list:
                robot_id = request.task_list[0].robot_id
                robot_namespace = f'robot_{robot_id}'
                if not self.robots[robot_namespace]["is_available"]:
                    # self.log_to_central('INFO', f'Robot {robot_id} is not available', robot_namespace, "handle_task_list")
                    response.success = False
                    return response

                waypoints = self.create_waypoints(request.task_list)
                self.robots[robot_namespace]["is_available"] = False
                self.robots[robot_namespace]["status"] = "busy"
                self.send_goal(robot_namespace, waypoints)
                response.success = True
                task_details = '\n'.join([f'Task: shelf_location=({task.shelf_location.position.x}, {task.shelf_location.position.y}) Shelf ID= {task.shelf_id}' for task in request.task_list])
                self.log_to_central('INFO', f'TaskList service called for {robot_namespace}. Task List:\n {task_details}', robot_namespace, "handle_task_list")
            else:
                self.log_to_central('INFO', 'No tasks available', log_source="handle_task_list")
                response.success = False
        except Exception as e:
            self.log_to_central('ERROR', f'Service call failed: {e}', log_source="handle_task_list")
            response.success = False
        return response

    def create_waypoints(self, task_list):
        waypoints = []
        for task in task_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose = task.shelf_location
            waypoints.append(pose)
        return waypoints

    def send_goal(self, robot_namespace, waypoints):
        client = ActionClient(self, FollowWaypoints, f'/{robot_namespace}/follow_waypoints')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        client.wait_for_server()

        goal_uuid = uuid.uuid4()
        self.robot_to_goal_id[str(goal_uuid)] = robot_namespace

        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, goal_id=str(goal_uuid): self.feedback_callback(feedback_msg, goal_id)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, robot_namespace, client))

    def feedback_callback(self, feedback_msg, goal_id):
        feedback = feedback_msg.feedback
        feedback_str = str(feedback)

        robot_namespace = self.robot_to_goal_id.get(goal_id)
        if not robot_namespace:
            self.log_to_central('ERROR', f'No robot namespace found for goal ID: {goal_id}', log_source="feedback_callback")
            return

        if robot_namespace not in self.previous_feedback:
            self.previous_feedback[robot_namespace] = None

        if feedback_str != self.previous_feedback[robot_namespace]:
            self.log_to_central('INFO', f'Robot {robot_namespace}: Feedback changed: {feedback_str}', robot_namespace, "feedback_callback")
            self.previous_feedback[robot_namespace] = feedback_str

    def goal_response_callback(self, future, robot_namespace, client):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_to_central('INFO', f'Goal rejected for {robot_namespace}', robot_namespace, "goal_response_callback")
            self.robots[robot_namespace]["is_available"] = True
            self.robots[robot_namespace]["status"] = "idle"
            return

        self.log_to_central('INFO', f'Goal accepted for {robot_namespace}', robot_namespace, "goal_response_callback")
        self.goal_handles[robot_namespace] = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, robot_namespace))

    def get_result_callback(self, future, robot_namespace):
        result = future.result().result
        self.log_to_central('INFO', f'Result received for {robot_namespace}: {result}', robot_namespace, "get_result_callback")
        for goal_id, stored_robot_namespace in list(self.robot_to_goal_id.items()):
          if stored_robot_namespace == robot_namespace:
              del self.robot_to_goal_id[goal_id]
        goal_handle = self.goal_handles.pop(robot_namespace, None)
        if goal_handle:
            # Update robot status to available after task completion
            self.robots[robot_namespace]["is_available"] = True
            self.robots[robot_namespace]["status"] = "idle"

def main(args=None):
    rclpy.init(args=args)
    
    # Create a temporary node to declare and retrieve parameters
    temp_node = rclpy.create_node('temp_node')
    temp_node.declare_parameter('num_robots', 2)  # Default value is 2
    num_robots = temp_node.get_parameter('num_robots').value
    temp_node.destroy_node()
    
    fleet_manager = FleetManager(num_robots)
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
