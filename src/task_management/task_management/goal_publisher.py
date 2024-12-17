import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # QoS profile for the /goal_pose topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher to the /goal_pose topic
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # Define waypoints (x, y, yaw) and waypoint names
        self.waypoints = [
            (-2.19105, -0.00456098, -0.00500371),
            (-1.11106, 1.71214, 0.0181757),
            (1.55431, 1.10929, -1.55228),
            (0.325672, 0.0265611, -1.59118),
            (1.13881, -1.38457, -1.60019),
            (-1.0675, -1.43716, -3.10911)
        ]
        self.waypoint_names = [f"Shelf_{i+1}" for i in range(len(self.waypoints))]

        # State variables
        self.current_goal_index = 0
        self.goal_reached = True  # Initially set to True to publish the first goal

        # Timer to periodically check and publish goals
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscription to /odom topic to verify goal reached
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.get_logger().info("Goal Publisher Node has been started.")


    def timer_callback(self):
        if self.goal_reached:
            self.publish_next_goal()

    def publish_next_goal(self):
        # Get the next goal
        x, y, yaw = self.waypoints[self.current_goal_index]
        waypoint_name = self.waypoint_names[self.current_goal_index]

        # Create and publish the PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = self.yaw_to_quaternion(yaw)

        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Published goal: {waypoint_name} at ({x}, {y}, {yaw})")

        # Move to the next goal
        self.current_goal_index = (self.current_goal_index + 1) % len(self.waypoints)
        self.goal_reached = False

    def odom_callback(self, msg):
        # Extract current robot position from Odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Get current goal
        goal_x, goal_y, _ = self.waypoints[self.current_goal_index - 1]

        # Compute distance to goal
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

        # Check if the robot is within a threshold distance of the goal
        if distance < 0.2:  # Adjust threshold as needed
            self.goal_reached = True

    @staticmethod
    def yaw_to_quaternion(yaw):
        # Convert yaw to quaternion (geometry_msgs/Quaternion)
        from geometry_msgs.msg import Quaternion
        half_yaw = yaw / 2.0
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
