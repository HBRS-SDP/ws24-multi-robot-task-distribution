import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from robot_interfaces.srv import GetRobotFleetStatus  # Ensure this is correctly defined
from robot_interfaces.msg import RobotStatus  # Import the custom message

class FleetManager(Node):
    def __init__(self, num_robots):
        super().__init__('fleet_manager')
        self.robots = {}

        # Initialize robots and subscriptions
        for i in range(1, num_robots + 1):
            robot_name = f'robot_{i}'
            odom_topic = f'/{robot_name}/odom'
            self.robots[robot_name] = {
                "id": i,
                "location": Point(),  # Using Point() as required
                "battery_level": 100.0,  # Placeholder
                "is_available": True,
                "status": "idle"
            }
            self.create_subscription(Odometry, odom_topic, lambda msg, name=robot_name: self.odom_callback(msg, name), 10)

        # Create a service to provide fleet status
        self.srv = self.create_service(GetRobotFleetStatus, 'get_robot_fleet_status', self.get_fleet_status)

    def odom_callback(self, msg, robot_name):
        if robot_name in self.robots:
            position = msg.pose.pose.position
            self.robots[robot_name]["location"].x = position.x
            self.robots[robot_name]["location"].y = position.y
            self.robots[robot_name]["location"].z = position.z
            self.get_logger().info(f'Updated {robot_name} location: ({position.x}, {position.y}, {position.z})')

    def get_fleet_status(self, request, response):
        response.robot_status_list = []
        for robot in self.robots.values():
            robot_status = RobotStatus()
            robot_status.robot_id = robot["id"]
            robot_status.current_location = robot["location"]
            robot_status.battery_level = robot["battery_level"]
            robot_status.is_available = robot["is_available"]
            robot_status.status = robot["status"]
            response.robot_status_list.append(robot_status)
        return response

def main(args=None):
    rclpy.init(args=args)
    num_robots = 2  # Change based on setup
    fleet_manager = FleetManager(num_robots)
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
