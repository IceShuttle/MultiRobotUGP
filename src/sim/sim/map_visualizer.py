#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

import pygame
import threading
import sys
import math

class MapVisualizer(Node):

    def __init__(self):
        super().__init__('map_visualizer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.entities_sub = self.create_subscription(
            MarkerArray,
            '/entities',
            self.entities_callback,
            10)
        self.map_data = None
        self.map_info = None
        self.cell_size = 25  # pixels per cell (2.5x bigger)
        self.screen = None
        self.clock = None
        self.running = False
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')

    def entities_callback(self, msg):
        """Update all entities from MarkerArray published by entity_sim."""
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []
        for marker in msg.markers:
            x = int(marker.pose.position.x / 0.05)
            y = int(marker.pose.position.y / 0.05)
            if marker.ns == 'humans':
                self.human_positions.append((x, y))
            elif marker.ns == 'fires':
                self.fire_positions.append((x, y))
            elif marker.ns == 'robots':
                self.robot_positions.append((x, y))
        self.get_logger().debug(f'Updated entities: {len(self.human_positions)}H, {len(self.fire_positions)}F, {len(self.robot_positions)}R')

    def run_pygame(self):
        while self.map_info is None and rclpy.ok():
            self.get_logger().info('Waiting for /map message...')
            pygame.time.wait(200)

        if self.map_info is None or not rclpy.ok():
            return

        try:
            pygame.init()
            w_px = self.map_info.width * self.cell_size
            h_px = self.map_info.height * self.cell_size
            self.screen = pygame.display.set_mode((w_px, h_px))
            pygame.display.set_caption('Map Visualizer')
            self.clock = pygame.time.Clock()
            self.running = True
            self.get_logger().info(f'Window opened ({w_px}x{h_px}). Close to quit.')

            while self.running and rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False

                rclpy.spin_once(self, timeout_sec=0.01)

                if self.map_data and self.map_info:
                    self.screen.fill((255, 255, 255))
                    # Draw occupied (black)
                    for y in range(self.map_info.height):
                        for x in range(self.map_info.width):
                            idx = y * self.map_info.width + x
                            if idx < len(self.map_data) and self.map_data[idx] > 50:
                                pygame.draw.rect(self.screen, (0, 0, 0),
                                                 (x * self.cell_size, y * self.cell_size,
                                                  self.cell_size, self.cell_size))
                    # Humans (blue)
                    for x, y in self.human_positions:
                        pygame.draw.circle(self.screen, (0, 100, 255),
                                           (x * self.cell_size + self.cell_size//2,
                                            y * self.cell_size + self.cell_size//2), self.cell_size//2 - 2)
                    # Fires (red rectangles)
                    for x, y in self.fire_positions:
                        pygame.draw.rect(self.screen, (255, 50, 0),
                                         (x * self.cell_size + 2, y * self.cell_size + 2,
                                          self.cell_size - 4, self.cell_size - 4))
                    # Robots (green circles)
                    for x, y in self.robot_positions:
                        pygame.draw.circle(self.screen, (50, 255, 50),
                                           (x * self.cell_size + self.cell_size//2,
                                            y * self.cell_size + self.cell_size//2),
                                           self.cell_size//2 - 2)
                    pygame.display.flip()

                self.clock.tick(15)
        except Exception as e:
            self.get_logger().error(f'Pygame error: {e}')
        finally:
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.run_pygame()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
