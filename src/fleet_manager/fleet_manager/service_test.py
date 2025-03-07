import rclpy
from rclpy.node import Node
from robot_interfaces.srv import GetRobotFleetStatus  # Import the service type

class FleetStatusClient(Node):
    def __init__(self):
        super().__init__('fleet_status_client')
        self.client = self.create_client(GetRobotFleetStatus, 'get_robot_fleet_status')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fleet manager service...')

        self.request_fleet_status()

    def request_fleet_status(self):
        request = GetRobotFleetStatus.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received fleet status:')
            for robot in response.robot_status_list:
                self.get_logger().info(
                    f"Robot {robot.robot_id}: Location ({robot.current_location.x}, "
                    f"{robot.current_location.y}, {robot.current_location.z}), "
                    f"Battery: {robot.battery_level}%, Available: {robot.is_available}, Status: {robot.status}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    fleet_client = FleetStatusClient()
    rclpy.spin(fleet_client)
    fleet_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
