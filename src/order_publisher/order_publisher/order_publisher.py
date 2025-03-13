import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order
from flask import Flask

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')

        # Create a publisher for the /order_requests topic
        self.publisher = self.create_publisher(Order, '/order_requests', 10)
        self.get_logger().info("Order Publisher Node is ready to send orders!")

    def publish_order(self, shelf_id, quantity):
        order_msg = Order()
        order_msg.shelf_id = shelf_id
        order_msg.quantity = quantity
        self.publisher.publish(order_msg)
        self.get_logger().info(f"Published Order: Shelf ID: {shelf_id}, Quantity: {quantity}")


def main(args=None):
    rclpy.init(args=args)
    order_publisher = OrderPublisher()
    
    rclpy.spin(order_publisher)
    order_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

