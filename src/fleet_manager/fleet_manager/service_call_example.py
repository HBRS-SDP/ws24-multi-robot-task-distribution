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
        tasks = []

        # Create multiple tasks
        for i in range(5):
            task = Task()
            task.task_id = 100 + i
            task.robot_id = 2
            task.shelf_id = 5 + i
            task.item = f'item_name_{i}'
            task.item_amount = 10 + i
            task.shelf_location = Point(x=1.2 , y=0.6 + i, z=0.0)
            task.task_type = 'pickup' if i % 2 == 0 else 'dropoff'
            tasks.append(task)
        
        self.request.task_list = tasks
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
