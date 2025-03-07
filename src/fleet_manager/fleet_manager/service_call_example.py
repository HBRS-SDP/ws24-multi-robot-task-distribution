import rclpy
from rclpy.node import Node
from robot_interfaces.srv import TaskList
from robot_interfaces.msg import Task
from geometry_msgs.msg import Point

class ServiceCaller(Node):

    def __init__(self):
        super().__init__('service_caller')
        self.client = self.create_client(TaskList, 'task_list')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TaskList.Request()

    def send_request(self):
        task = Task()
        task.task_id = 101
        task.robot_id = 1
        task.shelf_id = 5
        task.item = 'item_name'
        task.item_amount = 10
        task.shelf_location = Point(x=-1.5, y=4.6, z=0.0)
        task.task_type = 'pickup'
        
        self.request.task_list = [task]
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_caller = ServiceCaller()
    response = service_caller.send_request()
    service_caller.get_logger().info(f'Success: {response.success}')
    service_caller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
