import unittest
import pandas as pd
import os
from warehouse_manager.inventory_node import InventoryManager
import rclpy

class TestInventoryManager(unittest.TestCase):

    def setUp(self):
        """Setup test environment."""
        rclpy.init()
        self.node = InventoryManager()
        self.original_inventory = self.node.inventory.copy()

    def tearDown(self):
        """Cleanup after tests."""
        self.node.inventory = self.original_inventory
        self.node.save_inventory()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_load_inventory(self):
        """Test that inventory loads successfully."""
        self.assertIsNotNone(self.node.inventory, "Inventory should be loaded")

    def test_order_fulfillment(self):
        """Test that order fulfillment decreases stock."""
        initial_stock = self.node.inventory.copy()
        self.node.fulfill_order("A", 10)
        self.assertLess(self.node.inventory.at["A", "Shelf 1"], initial_stock.at["A", "Shelf 1"], "Stock should decrease")

    def test_out_of_stock_warning(self):
        """Test warning when stock is insufficient."""
        self.node.fulfill_order("A", 1000)  # Large number to deplete stock
        remaining_stock = self.node.inventory.loc["A"].sum()
        self.assertEqual(remaining_stock, 0, "Stock should be depleted")

if __name__ == '__main__':
    unittest.main()

