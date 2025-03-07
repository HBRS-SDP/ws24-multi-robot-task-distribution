import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QTableWidget, QTableWidgetItem, QHBoxLayout, QProgressBar
from robot_interfaces.srv import ShelfQuery, InventoryUpdate
from robot_interfaces.msg import Task, RobotStatus
from geometry_msgs.msg import Point

class Ros2Gui(Node):
    def __init__(self):
        super().__init__('ros2_gui')
        
        # Set up ROS 2 client for services
        self.database_client = self.create_client(ShelfQuery, '/database_query')
        self.inventory_update_client = self.create_client(InventoryUpdate, '/update_inventory')
        self.robot_status_subscriber = self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 10)
        
        # Wait for services to become available
        while not self.database_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Database service not available, waiting...')
        
        self.robot_status_data = {}

        # Set up the GUI using PyQt5
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Warehouse Management System")
        
        # Layouts
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()

        # Inventory Table
        self.inventory_table = QTableWidget(self)
        self.inventory_table.setColumnCount(5)
        self.inventory_table.setHorizontalHeaderLabels(["Shelf ID", "Product", "Capacity", "Inventory", "Location"])
        self.layout.addWidget(self.inventory_table)

        # Task Controls
        self.start_task_button = QPushButton("Start Task", self)
        self.end_task_button = QPushButton("End Task", self)
        self.button_layout.addWidget(self.start_task_button)
        self.button_layout.addWidget(self.end_task_button)

        self.layout.addLayout(self.button_layout)

        # Robot Status Table
        self.robot_status_table = QTableWidget(self)
        self.robot_status_table.setColumnCount(4)
        self.robot_status_table.setHorizontalHeaderLabels(["Robot ID", "Battery Level", "Location", "Status"])
        self.layout.addWidget(self.robot_status_table)

        self.window.setLayout(self.layout)
        self.window.show()

        # Connect buttons to callbacks
        self.start_task_button.clicked.connect(self.start_task)
        self.end_task_button.clicked.connect(self.end_task)

    def start_task(self):
        # Logic to start a task, send a message via the Task topic
        task_msg = Task()
        task_msg.robot_id = 1  # example robot ID
        task_msg.shelf_id = 2  # example shelf ID
        task_msg.task_type = "pickup"  # example task type
        self.get_logger().info("Starting task...")
        self.get_publisher(Task, '/task_assignments').publish(task_msg)

    def end_task(self):
        # Logic to end a task and update inventory
        task_msg = Task()
        task_msg.robot_id = 1  # example robot ID
        task_msg.shelf_id = 2  # example shelf ID
        task_msg.task_type = "pickup"  # example task type
        self.get_logger().info("Ending task...")
        self.get_publisher(Task, '/task_assignments').publish(task_msg)

        # Update Inventory after task completion
        self.update_inventory(2, 100)  # Update shelf with ID 2 and new inventory count (example)

    def update_inventory(self, shelf_id, new_inventory):
        # Send inventory update request
        inventory_request = InventoryUpdate()
        inventory_request.shelf_id = shelf_id
        inventory_request.new_inventory = new_inventory
        future = self.inventory_update_client.call_async(inventory_request)
        future.add_done_callback(self.handle_inventory_update)

    def handle_inventory_update(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info("Inventory updated successfully.")
        else:
            self.get_logger().warn("Failed to update inventory.")

    def robot_status_callback(self, msg):
        robot_id = msg.robot_id
        self.robot_status_data[robot_id] = {
            'battery_level': msg.battery_level,
            'location': msg.current_location,
            'status': msg.status
        }
        self.update_robot_status_table()

    def update_robot_status_table(self):
        self.robot_status_table.setRowCount(len(self.robot_status_data))
        row = 0
        for robot_id, status in self.robot_status_data.items():
            self.robot_status_table.setItem(row, 0, QTableWidgetItem(str(robot_id)))
            self.robot_status_table.setItem(row, 1, QTableWidgetItem(f"{status['battery_level']}%"))
            self.robot_status_table.setItem(row, 2, QTableWidgetItem(f"({status['location'].x}, {status['location'].y}, {status['location'].z})"))
            self.robot_status_table.setItem(row, 3, QTableWidgetItem(status['status']))
            row += 1

    def update_inventory_table(self, shelf_data):
        self.inventory_table.setRowCount(len(shelf_data))
        row = 0
        for shelf in shelf_data:
            self.inventory_table.setItem(row, 0, QTableWidgetItem(str(shelf['id'])))
            self.inventory_table.setItem(row, 1, QTableWidgetItem(shelf['product']))
            self.inventory_table.setItem(row, 2, QTableWidgetItem(str(shelf['capacity'])))
            self.inventory_table.setItem(row, 3, QTableWidgetItem(str(shelf['inventory'])))
            self.inventory_table.setItem(row, 4, QTableWidgetItem(f"({shelf['location'][0]}, {shelf['location'][1]}, {shelf['location'][2]})"))
            row += 1

    def update_shelves(self):
        # Query shelves from the shared memory node (database service)
        shelf_query = ShelfQuery()
        future = self.database_client.call_async(shelf_query)
        future.add_done_callback(self.handle_shelf_query)

    def handle_shelf_query(self, future):
        response = future.result()
        shelf_data = []
        for shelf in response.shelves:
            shelf_data.append({
                'id': shelf.id,
                'product': shelf.product,
                'capacity': shelf.capacity,
                'inventory': shelf.inventory,
                'location': [shelf.location.x, shelf.location.y, shelf.location.z]
            })
        self.update_inventory_table(shelf_data)

    def spin(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    ros2_gui = Ros2Gui()

    # Update shelves periodically
    ros2_gui.update_shelves()

    ros2_gui.spin()
    rclpy.shutdown()
    sys.exit(ros2_gui.app.exec_())

if __name__ == '__main__':
    main()

