#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger

import pygame
import threading

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
        self.move_pubs = [self.create_publisher(String, f'/robot{i}/move', 10) for i in range(4)]
        self.pick_clients = [self.create_client(Trigger, f'/robot{i}/pick') for i in range(4)]
        self.drop_clients = [self.create_client(Trigger, f'/robot{i}/drop') for i in range(4)]
        self.remove_fire_clients = [self.create_client(Trigger, f'/robot{i}/remove_fire') for i in range(4)]
        self.destroy_clients = [self.create_client(Trigger, f'/robot{i}/destroy') for i in range(4)]
        self.log_sub = self.create_subscription(String, '/action_log', self.log_callback, 10)
        self.action_log = []
        self.selected_robot = 0
        self.map_data = None
        self.map_info = None
        self.cell_size = 25
        self.screen = None
        self.clock = None
        self.running = False
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []
        self.robot_destroyed = []
        self.images = {}

    def create_images(self):
        size = self.cell_size
        half = size // 2

        robot_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.circle(robot_surf, (50, 200, 50), (half, half), half - 2)
        pygame.draw.circle(robot_surf, (30, 150, 30), (half, half), half - 2)
        pygame.draw.circle(robot_surf, (100, 255, 100), (half - 4, half - 4), 4)
        self.images['robot'] = robot_surf

        robot_destroyed_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.circle(robot_destroyed_surf, (128, 128, 128), (half, half), half - 2)
        pygame.draw.line(robot_destroyed_surf, (80, 80, 80), (4, 4), (size-4, size-4), 2)
        pygame.draw.line(robot_destroyed_surf, (80, 80, 80), (size-4, 4), (4, size-4), 2)
        self.images['robot_destroyed'] = robot_destroyed_surf

        human_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.circle(human_surf, (0, 100, 255), (half, half - 4), 6)
        pygame.draw.ellipse(human_surf, (0, 80, 200), (half - 6, half + 2, 12, 14))
        pygame.draw.line(human_surf, (0, 80, 200), (half - 4, half + 8), (half - 8, half + 16), 3)
        pygame.draw.line(human_surf, (0, 80, 200), (half + 4, half + 8), (half + 8, half + 16), 3)
        self.images['human'] = human_surf

        fire_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.rect(fire_surf, (255, 50, 0), (2, 2, size-4, size-4))
        pygame.draw.rect(fire_surf, (255, 150, 0), (4, 4, size-8, size-8))
        pygame.draw.circle(fire_surf, (255, 255, 100), (half, half), 5)
        self.images['fire'] = fire_surf

        floor_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        floor_surf.fill((40, 40, 50))
        for i in range(0, size, 6):
            pygame.draw.line(floor_surf, (50, 50, 60), (i, 0), (i, size))
            pygame.draw.line(floor_surf, (50, 50, 60), (0, i), (size, i))
        self.images['floor'] = floor_surf

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')

    def entities_callback(self, msg):
        """Update all entities from MarkerArray published by entity_sim."""
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []
        self.robot_destroyed = []
        for marker in msg.markers:
            x = int(marker.pose.position.x / 0.05)
            y = int(marker.pose.position.y / 0.05)
            if marker.ns == 'humans':
                self.human_positions.append((x, y))
            elif marker.ns == 'fires':
                self.fire_positions.append((x, y))
            elif marker.ns == 'robots':
                self.robot_positions.append((x, y))
                destroyed = (marker.color.r == 0.5 and marker.color.g == 0.5 and marker.color.b == 0.5)
                self.robot_destroyed.append(destroyed)
        self.get_logger().debug(f'Updated entities: {len(self.human_positions)}H, {len(self.fire_positions)}F, {len(self.robot_positions)}R')

    def move_robot(self, direction):
        msg = String()
        msg.data = direction
        self.move_pubs[self.selected_robot].publish(msg)

    def pick_human(self):
        if self.pick_clients[self.selected_robot].wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.pick_clients[self.selected_robot].call_async(req)

    def drop_human(self):
        if self.drop_clients[self.selected_robot].wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.drop_clients[self.selected_robot].call_async(req)

    def extinguish_fire(self):
        if self.remove_fire_clients[self.selected_robot].wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.remove_fire_clients[self.selected_robot].call_async(req)

    def log_callback(self, msg):
        self.action_log.append(msg.data)
        if len(self.action_log) > 10:
            self.action_log.pop(0)

    def run_pygame(self):
        while self.map_info is None and rclpy.ok():
            self.get_logger().info('Waiting for /map message...')
            pygame.time.wait(200)

        if self.map_info is None or not rclpy.ok():
            return

        try:
            pygame.init()
            pygame.font.init()
            self.create_images()
            self.panel_width = 300
            w_px = self.map_info.width * self.cell_size + self.panel_width
            h_px = self.map_info.height * self.cell_size
            self.screen = pygame.display.set_mode((w_px, h_px))
            pygame.display.set_caption('Map Visualizer')
            self.clock = pygame.time.Clock()
            self.running = True
            self.font = pygame.font.SysFont('Arial', 16)
            self.get_logger().info(f'Window opened ({w_px}x{h_px}). Close to quit.')

            while self.running and rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        mx, my = event.pos
                        panel_x = self.map_info.width * self.cell_size
                        if panel_x <= mx < panel_x + self.panel_width:
                            for i in range(4):
                                button_y = 50 + i * 60
                                if button_y <= my < button_y + 40:
                                    if self.destroy_clients[i].wait_for_service(timeout_sec=1.0):
                                        req = Trigger.Request()
                                        self.destroy_clients[i].call_async(req)
                                    break
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_1:
                            self.selected_robot = 0
                        elif event.key == pygame.K_2:
                            self.selected_robot = 1
                        elif event.key == pygame.K_3:
                            self.selected_robot = 2
                        elif event.key == pygame.K_4:
                            self.selected_robot = 3
                        elif event.key in [pygame.K_w, pygame.K_UP]:
                            self.move_robot('N')
                        elif event.key in [pygame.K_s, pygame.K_DOWN]:
                            self.move_robot('S')
                        elif event.key in [pygame.K_a, pygame.K_LEFT]:
                            self.move_robot('W')
                        elif event.key in [pygame.K_d, pygame.K_RIGHT]:
                            self.move_robot('E')
                        elif event.key == pygame.K_SPACE:
                            self.pick_human()
                        elif event.key == pygame.K_e:
                            self.drop_human()
                        elif event.key == pygame.K_r:
                            self.extinguish_fire()

                rclpy.spin_once(self, timeout_sec=0.01)

                if self.map_data and self.map_info:
                    for y in range(self.map_info.height):
                        for x in range(self.map_info.width):
                            px, py = x * self.cell_size, y * self.cell_size
                            idx = y * self.map_info.width + x
                            if idx < len(self.map_data) and self.map_data[idx] > 50:
                                pygame.draw.rect(self.screen, (20, 20, 20), (px, py, self.cell_size, self.cell_size))
                            else:
                                self.screen.blit(self.images['floor'], (px, py))

                    for x, y in self.human_positions:
                        self.screen.blit(self.images['human'], (x * self.cell_size, y * self.cell_size))

                    for x, y in self.fire_positions:
                        self.screen.blit(self.images['fire'], (x * self.cell_size, y * self.cell_size))

                    for i, (x, y) in enumerate(self.robot_positions):
                        img = self.images['robot_destroyed'] if self.robot_destroyed[i] else self.images['robot']
                        self.screen.blit(img, (x * self.cell_size, y * self.cell_size))

                # Draw control panel on right
                panel_x = self.map_info.width * self.cell_size
                pygame.draw.rect(self.screen, (220, 220, 220), (panel_x, 0, self.panel_width, h_px))
                # Left: buttons (140px)
                button_area_x = panel_x + 10
                sel_text = self.font.render(f'Selected: Robot {self.selected_robot}', True, (0, 0, 0))
                self.screen.blit(sel_text, (button_area_x, 10))
                for i in range(4):
                    button_y = 50 + i * 60
                    pygame.draw.rect(self.screen, (150, 150, 150), (button_area_x, button_y, 140, 40))
                    text = self.font.render(f'Destroy Robot {i}', True, (0, 0, 0))
                    self.screen.blit(text, (button_area_x + 10, button_y + 10))
                # Right: Action log (140px)
                log_area_x = panel_x + 160
                pygame.draw.line(self.screen, (0,0,0), (log_area_x - 10, 0), (log_area_x - 10, h_px))
                log_title = self.font.render('Action Log:', True, (0, 0, 0))
                self.screen.blit(log_title, (log_area_x, 10))
                log_y = 35
                start_idx = max(0, len(self.action_log) - 12)
                for log_entry in self.action_log[start_idx:]:
                    text = self.font.render(log_entry[:20], True, (0, 0, 0))  # truncate for width
                    self.screen.blit(text, (log_area_x, log_y))
                    log_y += 18
                    if log_y > h_px - 20:
                        break

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
