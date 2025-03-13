import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from robot_interfaces.msg import RobotStatus, FleetStatus
from robot_interfaces.srv import TaskList

class FleetManager(Node):
    def __init__(self, num_robots):
        super().__init__('fleet_manager')
        self.robots = {}
        self.goal_handles = {}

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

            self.robots["robot_1"]["battery_level"] = 50.0
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

    def odom_callback(self, msg, robot_name):
        if robot_name in self.robots:
            self.robots[robot_name]["location"] = msg.pose.pose
            position = msg.pose.pose.position
            # self.get_logger().info(f'Updated {robot_name} location: ({position.x}, {position.y}, {position.z})')

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
        self.get_logger().info('Published fleet status')

    def handle_task_list(self, request, response):
        try:
            if request.task_list:
                robot_id = request.task_list[0].robot_id
                robot_namespace = f'robot_{robot_id}'
                if not self.robots[robot_namespace]["is_available"]:
                    self.get_logger().info(f'Robot {robot_id} is not available')
                    response.success = False
                    return response

                waypoints = self.create_waypoints(request.task_list)
                self.robots[robot_namespace]["is_available"] = False
                self.robots[robot_namespace]["status"] = "busy"
                self.send_goal(robot_namespace, waypoints)
                response.success = True
            else:
                self.get_logger().info('No tasks available')
                response.success = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            response.success = False
        return response

    def create_waypoints(self, task_list):
        waypoints = []
        for task in task_list:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = task.shelf_location
   
            waypoints.append(pose_stamped)
        return waypoints

    def send_goal(self, robot_namespace, waypoints):
        self.client = ActionClient(self, FollowWaypoints, f'/{robot_namespace}/follow_waypoints')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, robot_namespace))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future, robot_namespace):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.robots[robot_namespace]["is_available"] = True
            self.robots[robot_namespace]["status"] = "idle"
            return

        self.get_logger().info('Goal accepted')
        self.goal_handles[robot_namespace] = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, robot_namespace))

    def get_result_callback(self, future, robot_namespace):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        goal_handle = self.goal_handles.pop(robot_namespace, None)
        if goal_handle:
            # Update robot status to available after task completion
            self.robots[robot_namespace]["is_available"] = True
            self.robots[robot_namespace]["status"] = "idle"

def main(args=None):
    rclpy.init(args=args)
    num_robots = 2  # Change based on setup
    fleet_manager = FleetManager(num_robots)
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()