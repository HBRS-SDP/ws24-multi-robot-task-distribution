import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from robot_interfaces.msg import RobotStatus, FleetStatus  # Import the custom messages

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
                "location": Pose(),  # Using Pose() as required
                "battery_level": 100.0,  # Placeholder
                "is_available": True,
                "status": "idle"
            }
            self.create_subscription(Odometry, odom_topic, lambda msg, name=robot_name: self.odom_callback(msg, name), 10)

        # Create a publisher for fleet status
        self.fleet_status_publisher = self.create_publisher(FleetStatus, 'fleet_status', 10)
        
        # Create a timer to periodically publish fleet status
        self.timer = self.create_timer(1.0, self.publish_fleet_status)  # Publish every second

    def odom_callback(self, msg, robot_name):
        if robot_name in self.robots:
            self.robots[robot_name]["location"] = msg.pose.pose
            position = msg.pose.pose.position
            self.get_logger().info(f'Updated {robot_name} location: ({position.x}, {position.y}, {position.z})')

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

def main(args=None):
    rclpy.init(args=args)
    num_robots = 2  # Change based on setup
    fleet_manager = FleetManager(num_robots)
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
