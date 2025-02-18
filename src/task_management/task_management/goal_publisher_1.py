import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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

        # Define 4 waypoints (x, y, yaw) for 4 shelves
        self.waypoints = [
            (-1.11106, 1.71214, 0.0181757),  # Shelf 1
            (1.55431, 1.10929, -1.55228),  # Shelf 2
            (-1.0675, -1.43716, -3.10911),  # Shelf 3
            (1.13881, -1.38457, -1.60019)  # Shelf 4
        ]
        self.waypoint_names = ["Shelf_1", "Shelf_2", "Shelf_3", "Shelf_4"]

        # Define the home position (original position)
        self.home_position = (0.325672, 0.0265611, -1.59118)

        # State variables
        self.current_goal_index = 0
        self.goal_reached = True  # Initially set to True to publish the first goal
        self.goals_published = 0  # Track how many goals have been published

        # Instructions
        self.client_orders = [
            {"client": "client1", "Shelf_1": "D", "Quantity_1": 19, "Shelf_2": "A", "Quantity_2": 4, "Shelf_3": "B", "Quantity_3": 23, "Shelf_4": "C", "Quantity_4": 11},
            {"client": "client2", "Shelf_1": "B", "Quantity_1": 16, "Shelf_2": "C", "Quantity_2": 8, "Shelf_3": "A", "Quantity_3": 1, "Shelf_4": "D", "Quantity_4": 20},
            {"client": "client3", "Shelf_1": "A", "Quantity_1": 17, "Shelf_2": "B", "Quantity_2": 7, "Shelf_3": "C", "Quantity_3": 5, "Shelf_4": "D", "Quantity_4": 14},
            {"client": "client4", "Shelf_1": "A", "Quantity_1": 20, "Shelf_2": "B", "Quantity_2": 8, "Shelf_3": "C", "Quantity_3": 19, "Shelf_4": "D", "Quantity_4": 11},
            {"client": "client5", "Shelf_1": "A", "Quantity_1": 23, "Shelf_2": "B", "Quantity_2": 9, "Shelf_3": "C", "Quantity_3": 11, "Shelf_4": "D", "Quantity_4": 12}
        ]
        self.current_client_index = 0
        self.current_order = self.client_orders[self.current_client_index]

        # Timer to periodically check and publish goals
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscription to /odom topic to verify goal reached
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.get_logger().info("Goal Publisher Node has been started.")

    def timer_callback(self):
        # Only publish the next goal if the previous one has been reached
        if self.goal_reached and self.goals_published < len(self.waypoints):
            self.publish_next_goal()

        # If all goals for the current client have been published, start returning to the original position
        if self.goals_published == len(self.waypoints):
            self.return_to_home()

    def publish_next_goal(self):
        # Get the next goal for the current order
        shelf_name = self.current_order.get(f"Shelf_{self.current_goal_index + 1}")
        quantity = self.current_order.get(f"Quantity_{self.current_goal_index + 1}")

        # Ensure the shelf name exists
        if shelf_name:
            shelf_index = self.waypoint_names.index(f"Shelf_{self.current_goal_index + 1}")
            x, y, yaw = self.waypoints[shelf_index]
            waypoint_name = f"Shelf_{self.current_goal_index + 1}"

            # Create and publish the PoseStamped message
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation = self.yaw_to_quaternion(yaw)

            self.publisher_.publish(goal_msg)
            self.get_logger().info(f"Robot 1 moving to {waypoint_name} for {quantity} items.")

            # Move to the next goal
            self.current_goal_index += 1
            self.goal_reached = False
            self.goals_published += 1

            # If all goals have been published for this client, log it
            if self.goals_published == len(self.waypoints):
                self.get_logger().info(f"All goals for {self.current_order['client']} published.")

    def return_to_home(self):
        # After all goals for the current client are published, return to home position
        home_x, home_y, home_yaw = self.home_position
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = home_x
        goal_msg.pose.position.y = home_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = self.yaw_to_quaternion(home_yaw)

        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Moving to drop-off location for {self.current_order['client']}.")

        # After reaching home, proceed to the next client
        self.current_client_index += 1
        if self.current_client_index < len(self.client_orders):
            self.current_order = self.client_orders[self.current_client_index]
            self.current_goal_index = 0
            self.goals_published = 0

    def odom_callback(self, msg):
        # Extract current robot position from Odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Get current goal
        if self.current_goal_index > 0:
            goal_x, goal_y, _ = self.waypoints[self.current_goal_index - 1]
        else:
            goal_x, goal_y, _ = self.home_position  # If it's the home position, we use that
            return  # No previous goal if none has been published

        # Compute distance to goal
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

        # Check if the robot is within a threshold distance of the goal
        if distance < 0.2:  # Adjust threshold as needed
            self.goal_reached = True
            if self.current_goal_index < len(self.waypoints):
                self.get_logger().info(f"Reached {self.waypoint_names[self.current_goal_index - 1]}.")
            else:
                self.get_logger().info(f"Reached drop-off location for {self.current_order['client']}.")

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

