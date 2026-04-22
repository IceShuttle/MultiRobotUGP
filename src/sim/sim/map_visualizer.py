#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid

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
        self.map_data = None
        self.map_info = None
        self.cell_size = 25  # pixels per cell (2.5x bigger)
        self.screen = None
        self.clock = None
        self.running = False

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')

    def run_pygame(self):
        while self.map_info is None and rclpy.ok():
            self.get_logger().info('Waiting for /map message...')
            pygame.time.wait(500)
            continue

        if not rclpy.ok():
            return

        try:
            pygame.init()
            width_px = self.map_info.width * self.cell_size
            height_px = self.map_info.height * self.cell_size
            self.screen = pygame.display.set_mode((width_px, height_px))
            pygame.display.set_caption('Map Visualizer')
            self.clock = pygame.time.Clock()
            self.running = True
            self.get_logger().info(f'Pygame window opened ({width_px}x{height_px}). Close to quit.')

            while self.running and rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False

                if self.map_data and self.map_info:
                    self.screen.fill((255, 255, 255))
                    for y in range(self.map_info.height):
                        for x in range(self.map_info.width):
                            idx = y * self.map_info.width + x
                            if idx < len(self.map_data) and self.map_data[idx] > 50:
                                pygame.draw.rect(self.screen, (0, 0, 0),
                                                 (x * self.cell_size, y * self.cell_size,
                                                  self.cell_size, self.cell_size))
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
