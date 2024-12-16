import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import tf_transformations
from tf2_geometry_msgs import PointStamped

class PotentialAStarNavigation(Node):
    def __init__(self):
        super().__init__('potential_field_avoidance')
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'ground_truth_pose', self.pose_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_speed = 0.9
        self.k_rep = 0.5  # Repulsive coefficient
        self.k_att = 1.0  # Attractive coefficient
        self.goal = np.array([[4.0, 10.0, -1.0], [2.0, 5.0, -2.0], [-4.0, 5.0, 1.0], [4.0, -10.0, 1.57]])  # Goal position (x, y, theta) in odom frame
        self.goal_index = 0
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Placeholder for the current pose
    
    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Convert quaternion to Euler angles
        orientation = msg.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        theta = euler[2]  # Yaw
        self.get_logger().info(f'Received goal_pose: x={x}, y={y}, theta={theta}')
        # self.goal = np.array([x, y, theta])

    def pose_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        self.current_pose['theta'] = 2.0 * np.arctan2(orientation_z, orientation_w)
    
    def scan_callback(self, msg):
        if self.current_pose is None:
            return  # Wait until we have the current pose

        # Transform goal position to base_link frame
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            goal_point = PointStamped()
            goal_point.header.frame_id = 'odom'
            goal_point.point.x = self.goal[self.goal_index][0]
            goal_point.point.y = self.goal[self.goal_index][1]
            goal_point.point.z = 0.0
            transformed_goal = self.tf_buffer.transform(goal_point, 'base_link')
            goal_base_link = np.array([transformed_goal.point.x, transformed_goal.point.y])
        except Exception as e:
            self.get_logger().warn(f'Failed to transform goal to base_link frame: {e}')
            return

        obstacle_forces = self.calculate_repulsive_forces(msg)
        attractive_force = self.calculate_attractive_force(goal_base_link)
        total_force = attractive_force + obstacle_forces

        self.get_logger().info(f'Attractive force: {attractive_force}, Repulsive force: {obstacle_forces}, Total force: {total_force}')
        self.move_robot(total_force, goal_base_link)

    def calculate_repulsive_forces(self, scan):
        forces = np.zeros(2)
        for i, range in enumerate(scan.ranges):
            if np.isinf(range) or range >= scan.range_max:
                continue  # Skip if the range is infinite or beyond the maximum range
            angle = scan.angle_min + i * scan.angle_increment
            force_direction = np.array([np.cos(angle), np.sin(angle)])
            force_magnitude = self.k_rep * (1.0 / range - 1.0 / scan.range_max) / (range ** 2)
            forces -= force_magnitude * force_direction

        return forces

    def calculate_attractive_force(self, goal_base_link):
        direction_to_goal = goal_base_link
        distance_to_goal = np.linalg.norm(direction_to_goal)
        if distance_to_goal < 0.5:
            # Control orientation when close to goal
            force_magnitude = 0.0
            theta_goal = self.goal[self.goal_index][2]
            current_yaw = self.current_pose['theta']
            angular_error = self.normalize_angle(theta_goal - current_yaw)
            return np.array([0.0, angular_error])
        else:
            # Normal attractive force
            force_magnitude = self.k_att * distance_to_goal
            force_direction = direction_to_goal / (distance_to_goal + 1e-6)  # Avoid division by zero
            return force_magnitude * force_direction

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def move_robot(self, force, goal_base_link):
        msg = Twist()
        force_magnitude = np.linalg.norm(force)
        distance_to_goal = np.linalg.norm(goal_base_link)
        
        if distance_to_goal < 0.1:
            # Stop the robot if it is very close to the goal
            # m
            self.goal_index += 1
            print(self.goal_index)
            if self.goal_index == len(self.goal):
                print('Sucess')
                rclpy.shutdown()
        elif force_magnitude < 0.1 and self.goal_index == len(self.goal) - 1:  # When close to goal, just rotate to desired orientation
            msg.linear.x = 0.0
            msg.angular.z = force[1]
        else:
            msg.linear.x = min(self.robot_speed, distance_to_goal)
            msg.angular.z = np.clip(np.arctan2(force[1], force[0]), -1.0, 1.0)  # Clipping angular velocity

        self.get_logger().info(f'Moving with linear x: {msg.linear.x} angular z: {msg.angular.z}')
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialAStarNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()