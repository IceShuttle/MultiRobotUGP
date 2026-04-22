#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory

import os
import csv
import math

class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(5.0, self.publish_map)
        self.get_logger().info('Map publisher node started. Place map.csv in src/sim/ before building.')
        self.map_msg = None
        self.load_map()

    def load_map(self):
        share_dir = get_package_share_directory('sim')
        csv_path = os.path.join(share_dir, 'map.csv')
        if not os.path.exists(csv_path):
            self.get_logger().warn(f'Map file not found at {csv_path}')
            return

        grid = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    grid.append([int(float(x)) for x in row])

        if not grid or not grid[0]:
            self.get_logger().warn('Invalid map.csv')
            return

        height = len(grid)
        width = len(grid[0])

        # Verify all rows have same width
        for row in grid:
            if len(row) != width:
                self.get_logger().error('All rows in CSV must have same length')
                return

        data = []
        for row in grid:
            for cell in row:
                data.append(100 if cell == 1 else 0)

        resolution = 0.05
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        self.map_msg.info.height = height
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = data
        self.get_logger().info(f'Map loaded: {width}x{height} from CSV (resolution={resolution}m)')

    def publish_map(self):
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
