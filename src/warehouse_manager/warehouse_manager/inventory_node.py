import rclpy
from rclpy.node import Node
import pandas as pd
import os
import datetime

class InventoryManager(Node):
    def __init__(self):
        super().__init__('inventory_manager')
        self.inventory_file = os.path.join(os.path.dirname(__file__), 'inventory.csv')
        self.orders_file = os.path.join(os.path.dirname(__file__), 'orders.csv')
        self.log_file = os.path.join(os.path.dirname(__file__), 'logs.csv')
        self.load_inventory()
        self.process_orders()

    def load_inventory(self):
        """Loads inventory from CSV file."""
        if os.path.exists(self.inventory_file):
            self.inventory = pd.read_csv(self.inventory_file, index_col=0)
            self.get_logger().info('Inventory loaded successfully.')
        else:
            self.get_logger().error('Inventory file not found!')
            self.inventory = None

    def save_inventory(self):
        """Saves updated inventory to CSV file."""
        self.inventory.to_csv(self.inventory_file)
        self.get_logger().info('Inventory updated and saved.')

    def log_action(self, order_id, product, shelf, quantity, status):
        """Logs the picking action to logs.csv"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = pd.DataFrame([[timestamp, order_id, product, shelf, quantity, status]], 
                                  columns=["Timestamp", "Order ID", "Product", "Shelf", "Quantity Picked", "Status"])
        if os.path.exists(self.log_file):
            log_entry.to_csv(self.log_file, mode='a', header=False, index=False)
        else:
            log_entry.to_csv(self.log_file, mode='w', header=True, index=False)

    def process_orders(self):
        """Processes orders from CSV file and updates inventory."""
        if not os.path.exists(self.orders_file):
            self.get_logger().error('Orders file not found!')
            return

        orders = pd.read_csv(self.orders_file)
        for _, order in orders.iterrows():
            order_id = order["Order ID"]
            input(f"\n[INFO] Processing Order ID: {order_id} - Press Enter to continue...")
            self.get_logger().info(f'Processing Order ID: {order_id}')
            for col in range(1, len(order), 2):  # Loop through product-quantity pairs
                product = order[col]
                qty_needed = order[col + 1]
                if pd.isna(product) or pd.isna(qty_needed):
                    continue
                
                if qty_needed == 0:
                    self.get_logger().info(f'Picked 0 units of {product}')
                    self.log_action(order_id, product, "N/A", 0, "Picked 0")
                else:
                    self.fulfill_order(order_id, product, qty_needed)
        
        self.save_inventory()

    def fulfill_order(self, order_id, product, qty_needed):
        """Fulfills an order by picking items from shelves sequentially."""
        if product not in self.inventory.index:
            self.get_logger().warn(f'Product {product} not found in inventory!')
            self.log_action(order_id, product, "N/A", 0, "Product Not Found")
            return

        self.get_logger().info(f'Picking {qty_needed} units of {product}')
        total_picked = 0
        
        for shelf in self.inventory.columns:
            if qty_needed <= 0:
                break
            
            available = self.inventory.at[product, shelf]
            if available > 0:
                pick = min(available, qty_needed)
                self.inventory.at[product, shelf] -= pick
                qty_needed -= pick
                total_picked += pick
                self.get_logger().info(f'Picked {pick} from {shelf}, Remaining to pick: {qty_needed}')
                
                # Log the action
                self.log_action(order_id, product, shelf, pick, "Fulfilled")

        if total_picked == 0:
            self.get_logger().warn(f'No stock available for {product}!')
            self.log_action(order_id, product, "N/A", 0, "Picked 0")
        elif qty_needed > 0:
            self.get_logger().warn(f'Not enough stock for {product}, {qty_needed} units unfulfilled!')
            self.log_action(order_id, product, "N/A", qty_needed, "Partial Fulfillment")


def main(args=None):
    rclpy.init(args=args)
    node = InventoryManager()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

