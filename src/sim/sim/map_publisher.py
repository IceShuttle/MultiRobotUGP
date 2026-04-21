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
            self.get_logger().warn(f'Map file not found at {csv_path}. Create map.csv with x,y occupied points.')
            return
        points = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        x, y = float(row[0]), float(row[1])
                        points.append((x, y))
                    except ValueError:
                        continue
        if not points:
            self.get_logger().warn('No valid points in CSV.')
            return

        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_y = min(p[1] for p in points)
        max_y = max(p[1] for p in points)

        resolution = 0.05  # 5cm
        width = int(math.ceil((max_x - min_x) / resolution)) + 1
        height = int(math.ceil((max_y - min_y) / resolution)) + 1

        data = [0] * (width * height)

        for x, y in points:
            ix = int((x - min_x) / resolution)
            iy = int((y - min_y) / resolution)
            if 0 <= ix < width and 0 <= iy < height:
                data[iy * width + ix] = 100

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        self.map_msg.info.height = height
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position.x = min_x - resolution / 2
        self.map_msg.info.origin.position.y = min_y - resolution / 2
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = data
        self.get_logger().info(f'Map loaded: {width}x{height} cells, {len(points)} obstacles.')

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
