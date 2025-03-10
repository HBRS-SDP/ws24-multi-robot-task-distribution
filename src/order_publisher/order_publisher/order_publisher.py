import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order  # Assuming Order is defined in robot_interfaces.msg
from rclpy.parameter import Parameter


class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')

        # Create a publisher for the /order_requests topic
        self.publisher = self.create_publisher(Order, '/order_requests', 10)

        # Timer to periodically send orders (you can remove this and add a manual input later)
        self.timer = self.create_timer(5.0, self.send_order)  # Send order every 5 seconds for testing

        self.get_logger().info("Order Publisher Node is ready to send orders!")

    def send_order(self):
        # Example: Manually send a custom order
        shelf_id = 1  # Example shelf id
        quantity = 10  # Example quantity

        order_msg = Order()  # Create an Order message
        order_msg.shelf_id = shelf_id
        order_msg.quantity = quantity

        # Publish the order
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
