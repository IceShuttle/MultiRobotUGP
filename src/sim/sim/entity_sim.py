#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import random

class EntitySim(Node):
    def __init__(self):
        super().__init__('entity_sim')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.entity_pub = self.create_publisher(MarkerArray, '/entities', 10)
        self.timer = self.create_timer(2.0, self.publish_entities)
        self.map_info = None
        self.free_cells = []
        self.human_positions = []
        self.fire_positions = []
        self.get_logger().info('EntitySim started. Waiting for map...')

    def map_callback(self, msg):
        if self.map_info is not None:
            return  # only process first map
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        self.free_cells = []
        for y in range(height):
            for x in range(width):
                if data[y * width + x] <= 0:  # free
                    self.free_cells.append((x, y))

        if len(self.free_cells) < 8:
            self.get_logger().warn('Not enough free cells')
            return

        random.shuffle(self.free_cells)
        self.human_positions = self.free_cells[:5]   # 5 humans
        self.fire_positions = self.free_cells[5:8]   # 3 fires

        self.get_logger().info(f'Placed {len(self.human_positions)} humans and {len(self.fire_positions)} fires on free cells')
        self.publish_entities()

    def publish_entities(self):
        if not self.human_positions and not self.fire_positions:
            return

        markers = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        # Humans - blue cylinders
        for i, (x, y) in enumerate(self.human_positions):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = timestamp
            marker.ns = 'humans'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x * 0.05 + 0.025
            marker.pose.position.y = y * 0.05 + 0.025
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.0
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            markers.markers.append(marker)

        # Fires - red spheres
        for i, (x, y) in enumerate(self.fire_positions):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = timestamp
            marker.ns = 'fires'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x * 0.05 + 0.025
            marker.pose.position.y = y * 0.05 + 0.025
            marker.pose.position.z = 0.3
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.9)
            markers.markers.append(marker)

        self.entity_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = EntitySim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
