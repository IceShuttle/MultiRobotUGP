#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger

import heapq
import math
import time
import random
from collections import defaultdict

class Solver(Node):
    def __init__(self):
        super().__init__('solver')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.entities_sub = self.create_subscription(MarkerArray, '/entities', self.entities_callback, 10)
        self.move_pubs = [self.create_publisher(String, f'/robot{i}/move', 10) for i in range(4)]
        self.pick_clients = [self.create_client(Trigger, f'/robot{i}/pick') for i in range(4)]
        self.drop_clients = [self.create_client(Trigger, f'/robot{i}/drop') for i in range(4)]
        self.map_data = None
        self.map_info = None
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []
        self.get_logger().info('Solver started. Waiting for map and entities...')

    def map_callback(self, msg):
        self.map_data = list(msg.data)
        self.map_info = msg.info
        self.get_logger().info('Map received by solver')

    def entities_callback(self, msg):
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
        if self.map_info and len(self.robot_positions) == 4:
            self.get_logger().info('All data received. Starting naive solver...')
            self.solve()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        if self.map_info is None or self.map_data is None:
            return []
        width = self.map_info.width
        height = self.map_info.height
        obstacles = set(self.fire_positions + self.robot_positions)
        if goal in obstacles:
            obstacles.remove(goal)  # allow goal
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0
        f_score = defaultdict(lambda: float('inf'))
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                    idx = neighbor[1] * width + neighbor[0]
                    if self.map_data[idx] > 50 or neighbor in obstacles:
                        continue
                    tentative_g = g_score[current] + 1
                    if tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        if neighbor not in [i[1] for i in open_set]:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def solve(self):
        if len(self.robot_positions) != 4:
            self.get_logger().warn('Not enough data for solver')
            return

        # Divide map into 4 zones (quadrants)
        w = self.map_info.width // 2
        h = self.map_info.height // 2
        zones = [
            (0, 0, w, h),      # Zone 0: top-left
            (w, 0, self.map_info.width, h),   # Zone 1: top-right
            (0, h, w, self.map_info.height),  # Zone 2: bottom-left
            (w, h, self.map_info.width, self.map_info.height)  # Zone 3: bottom-right
        ]

        # Assign closest human in zone to each robot (strict per quadrant, no fallback)
        assignments = []
        for i in range(4):
            rx, ry = self.robot_positions[i]
            zone = zones[i]
            zone_humans = [h for h in self.human_positions if zone[0] <= h[0] < zone[2] and zone[1] <= h[1] < zone[3]]
            if zone_humans:
                closest = min(zone_humans, key=lambda h: abs(h[0]-rx) + abs(h[1]-ry))
                assignments.append((i, closest))
                self.get_logger().info(f'Robot {i} assigned human at {closest} in its quadrant')
            else:
                self.get_logger().info(f'Robot {i} quadrant has no humans')

        self.get_logger().info(f'Assigned {len(assignments)} humans to robots (strict quadrants)')

        # Parallel rescue (all robots move together)
        import threading
        threads = []
        for robot_id, human_pos in assignments:
            t = threading.Thread(target=self.robot_rescue_task, args=(robot_id, human_pos))
            threads.append(t)
            t.start()
        for t in threads:
            t.join()

        self.get_logger().info('All humans rescued. Solver complete.')

    def robot_rescue_task(self, robot_id, human_pos):
        self.get_logger().info(f'Robot {robot_id} going for human at {human_pos}')
        path = self.a_star(self.robot_positions[robot_id], human_pos)
        self.follow_path(robot_id, path)
        self.pick(robot_id)
        # Find nearest empty edge cell for drop
        edge_cells = [(x, y) for y in range(self.map_info.height) for x in range(self.map_info.width)
                      if (x == 0 or x == self.map_info.width-1 or y == 0 or y == self.map_info.height-1) and
                         (x, y) not in self.robot_positions and (x, y) not in self.fire_positions and (x, y) not in self.human_positions]
        if edge_cells:
            nearest_edge = min(edge_cells, key=lambda e: abs(e[0] - self.robot_positions[robot_id][0]) + abs(e[1] - self.robot_positions[robot_id][1]))
            self.get_logger().info(f'Robot {robot_id} going to exit at {nearest_edge}')
            path = self.a_star(self.robot_positions[robot_id], nearest_edge)
            self.follow_path(robot_id, path)
            self.drop(robot_id)
        else:
            self.get_logger().warn(f'Robot {robot_id} no free edge for drop')



    def follow_path(self, robot_id, path):
        for pos in path:
            dx = pos[0] - self.robot_positions[robot_id][0]
            dy = pos[1] - self.robot_positions[robot_id][1]
            if dx == 1:
                dir = 'E'
            elif dx == -1:
                dir = 'W'
            elif dy == 1:
                dir = 'S'
            elif dy == -1:
                dir = 'N'
            else:
                continue
            msg = String()
            msg.data = dir
            self.move_pubs[robot_id].publish(msg)
            self.robot_positions[robot_id] = pos
            time.sleep(0.3)  # simulate movement time

    def pick(self, robot_id):
        if self.pick_clients[robot_id].wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            self.pick_clients[robot_id].call_async(req)
            time.sleep(0.5)

    def drop(self, robot_id):
        if self.drop_clients[robot_id].wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            self.drop_clients[robot_id].call_async(req)
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = Solver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
