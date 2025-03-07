import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import TaskList

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.srv = self.create_service(TaskList, 'task_list', self.handle_task_list)

    def handle_task_list(self, request, response):
        try:
            if request.task_list:
                robot_id = request.task_list[0].robot_id
                robot_namespace = f'robot_{robot_id}'
                waypoints = self.create_waypoints(request.task_list)
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
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = task.shelf_location.x
            pose.pose.position.y = task.shelf_location.y
            pose.pose.position.z = task.shelf_location.z
            pose.pose.orientation.w = 1.0  # Assuming no rotation
            waypoints.append(pose)
        return waypoints

    def send_goal(self, robot_namespace, waypoints):
        self.client = ActionClient(self, FollowWaypoints, f'/{robot_namespace}/follow_waypoints')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

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

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
