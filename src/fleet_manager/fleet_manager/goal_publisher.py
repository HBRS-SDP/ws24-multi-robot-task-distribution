import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import csv

import pkg_resources

class WaypointFollower(Node):
    def __init__(self, robot_namespace):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, FollowWaypoints, f'/{robot_namespace}/follow_waypoints')

    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def create_waypoint(x, y, z=0.0, q_w=1.0, q_x=0.0, q_y=0.0, q_z=0.0):
    waypoint = PoseStamped()
    waypoint.header.frame_id = 'map'
    waypoint.pose.position.x = x
    waypoint.pose.position.y = y
    waypoint.pose.position.z = z
    waypoint.pose.orientation.w = q_w
    waypoint.pose.orientation.x = q_x
    waypoint.pose.orientation.y = q_y
    waypoint.pose.orientation.z = q_z
    return waypoint

def read_shelf_positions(csv_file, order):
    waypoints = []
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        for shelf in order:
            for row in rows:
                if int(row['Shelf']) == shelf:
                    x = float(row['X'])
                    y = float(row['Y'])
                    z = float(row['Z'])
                    q_w = float(row['Q_W'])
                    q_x = float(row['Q_X'])
                    q_y = float(row['Q_Y'])
                    q_z = float(row['Q_Z'])
                    waypoints.append(create_waypoint(x, y, z, q_w, q_x, q_y, q_z))
                    break
    return waypoints

def main():
    rclpy.init()
    robot_namespace = 'robot_1'  # You can change this to any robot namespace
    node = WaypointFollower(robot_namespace)
    csv_file = pkg_resources.resource_filename("fleet_manager", 'shelf_pose.csv')
    order = [3, 6, 1]  # Order of shelves to follow
    waypoints = read_shelf_positions(csv_file, order)
    node.send_goal(waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
