import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from robot_interfaces.action import Task
from rclpy.action import ActionServer

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.task_publisher = self.create_publisher(Task.Goal, 'task_topic', 10)
        self._action_server = ActionServer(
            self,
            Task,
            'assign_task',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        task_msg = Task.Goal()
        task_msg.robot_id = goal_handle.request.robot_id
        task_msg.shelf_id = goal_handle.request.shelf_id
        task_msg.shelf_location = goal_handle.request.shelf_location
        task_msg.task_type = goal_handle.request.task_type
        self.task_publisher.publish(task_msg)
        goal_handle.succeed()
        result = Task.Result()
        result.success = True
        return result

    def send_goal(self, robot_id, goal_pose):
        goal_topic = f'{robot_id}/goal_pose'
        goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        goal_publisher.publish(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    fleet_manager = FleetManager()
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()