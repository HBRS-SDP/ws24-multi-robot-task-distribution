import rclpy
from rclpy.node import Node
import csv
from shared_memory_msgs.msg import GoalStartMsg, GoalReachMsg
from shared_memory_msgs.srv import GetRobotState, GetInventoryStatus

class SharedMemoryNode(Node):
    def __init__(self):
        super().__init__('shared_memory_node')

        # Load inventory data from CSV
        self.inventory = self.load_inventory_data()

        # Robot status storage
        self.robots = {}

        # Subscribers
        self.create_subscription(GoalStartMsg, '/goal_start', self.log_goal_start, 10)
        self.create_subscription(GoalReachMsg, '/goal_reach', self.log_goal_reach, 10)

        # Services
        self.create_service(GetRobotState, '/get_robot_state', self.get_robot_state)
        self.create_service(GetInventoryStatus, '/get_inventory_status', self.get_inventory_status)

        self.get_logger().info("Shared Memory Node is running...")

    def load_inventory_data(self):
        """Reads inventory data from the CSV file and stores it in a dictionary."""
        inventory = {}

        # Define CSV file path
        csv_file_path = "/home/behrouz/ws24-multi-robot-task-distribution/src/shared_memory/shared_memory_node/inventory_data/inventory.csv"

        try:
            with open(csv_file_path, 'r') as file:
                reader = csv.reader(file)
                rows = list(reader)

                # Extract shelf IDs and product quantities
                shelves = rows[0][1:]  # Shelf 1 to Shelf 8
                products = rows[1][1:]  # Product names (A to H)
                quantities = list(map(int, rows[2][1:]))  # Convert quantities to integers

                # Store inventory as { shelf_name: { product: quantity } }
                for i in range(len(shelves)):
                    inventory[shelves[i]] = {
                        "product": products[i],
                        "init": quantities[i],
                        "remaining": quantities[i]
                    }

            self.get_logger().info("Inventory successfully loaded from CSV.")
        except Exception as e:
            self.get_logger().error(f"Error loading inventory from CSV: {e}")

        return inventory

    def log_goal_start(self, msg):
        """Updates robot status when a goal starts."""
        self.get_logger().info(f"Robot {msg.robot_id} started task at {msg.shelf} for client {msg.client_id}")
        self.robots[msg.robot_id] = {'status': 'moving', 'shelf': msg.shelf, 'items': msg.items}

    def log_goal_reach(self, msg):
        """Updates robot status when it reaches a shelf."""
        self.get_logger().info(f"Robot {msg.robot_id} reached {msg.shelf}")

        if msg.shelf in self.inventory:
            self.inventory[msg.shelf]["remaining"] -= self.robots[msg.robot_id]["items"]
            self.get_logger().info(f"📉 Inventory updated: {self.inventory[msg.shelf]['remaining']} items left on {msg.shelf}")

        self.robots[msg.robot_id]['status'] = 'idle'

    def get_robot_state(self, request, response):
        """Returns the current state of a requested robot."""
        robot = self.robots.get(request.robot_id, {'shelf': 'unknown', 'items': 0})
        response.robot_id = request.robot_id
        response.shelf = robot.get('shelf', 'unknown')
        response.items = robot.get('items', 0)
        return response

    def get_inventory_status(self, request, response):
        """Returns the current inventory status for a given shelf."""
        shelf_data = self.inventory.get(request.shelf, {'init': 0, 'remaining': 0})
        response.init_quantity = shelf_data['init']
        response.remaining_quantity = shelf_data['remaining']
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SharedMemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

