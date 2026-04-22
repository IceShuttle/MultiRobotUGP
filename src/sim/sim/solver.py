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
import threading
from collections import defaultdict


class Solver(Node):
    def __init__(self):
        super().__init__('solver')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.entities_sub = self.create_subscription(
            MarkerArray, '/entities', self.entities_callback, 10)
        self.move_pubs = [
            self.create_publisher(String, f'/robot{i}/move', 10) for i in range(4)]
        self.pick_clients = [
            self.create_client(Trigger, f'/robot{i}/pick') for i in range(4)]
        self.drop_clients = [
            self.create_client(Trigger, f'/robot{i}/drop') for i in range(4)]

        self.map_data = None
        self.map_info = None

        # Live positions coming from entity_sim via /entities topic
        self.live_human_positions = []
        self.live_fire_positions = []
        self.live_robot_positions = []

        # Internal tracked positions used by the solver threads
        self.robot_pos = [None] * 4
        self.robot_dir = [None] * 4

        self.lock = threading.Lock()
        self.solved = False

        self.get_logger().info('Solver started. Waiting for map and entities...')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def map_callback(self, msg):
        self.map_data = list(msg.data)
        self.map_info = msg.info
        self.get_logger().info('Map received by solver')

    def entities_callback(self, msg):
        humans, fires, robots = [], [], []
        for marker in msg.markers:
            x = int(marker.pose.position.x / 0.05)
            y = int(marker.pose.position.y / 0.05)
            if marker.ns == 'humans':
                humans.append((x, y))
            elif marker.ns == 'fires':
                fires.append((x, y))
            elif marker.ns == 'robots':
                robots.append((x, y))

        self.live_human_positions = humans
        self.live_fire_positions = fires
        self.live_robot_positions = robots

        if not self.solved and self.map_info and len(robots) == 4:
            self.solved = True
            with self.lock:
                for i in range(4):
                    self.robot_pos[i] = robots[i]
                    self.robot_dir[i] = None
            self.get_logger().info('All data received. Launching solver...')
            t = threading.Thread(target=self.solve, daemon=True)
            t.start()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def euclidean(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def is_wall(self, x, y):
        if not (0 <= x < self.map_info.width and 0 <= y < self.map_info.height):
            return True
        return self.map_data[y * self.map_info.width + x] > 50

    def get_edge_cells(self):
        w, h = self.map_info.width, self.map_info.height
        cells = []
        for x in range(w):
            for y in range(h):
                if x == 0 or x == w - 1 or y == 0 or y == h - 1:
                    if not self.is_wall(x, y):
                        cells.append((x, y))
        return cells

    # ------------------------------------------------------------------
    # A* -- treats other robots + heading cell as dynamic obstacles
    # ------------------------------------------------------------------
    def a_star(self, start, goal, robot_id):
        if self.map_info is None or self.map_data is None:
            return []

        width = self.map_info.width
        height = self.map_info.height

        with self.lock:
            dynamic = set()
            for i in range(4):
                if i == robot_id:
                    continue
                pos = self.robot_pos[i]
                if pos is None:
                    continue
                dynamic.add(pos)
                d = self.robot_dir[i]
                if d == 'N' and pos[1] > 0:
                    dynamic.add((pos[0], pos[1] - 1))
                elif d == 'S' and pos[1] < height - 1:
                    dynamic.add((pos[0], pos[1] + 1))
                elif d == 'E' and pos[0] < width - 1:
                    dynamic.add((pos[0] + 1, pos[1]))
                elif d == 'W' and pos[0] > 0:
                    dynamic.add((pos[0] - 1, pos[1]))

        obstacles = set(self.live_fire_positions) | set(self.live_human_positions) | dynamic
        obstacles.discard(goal)
        obstacles.discard(start)

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0.0
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if self.map_data[ny * width + nx] > 50:
                    continue
                if neighbor in obstacles:
                    continue
                tentative_g = g_score[current] + 1.0
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.euclidean(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return []

    # A* ignoring other robots entirely -- fallback when blocked
    def a_star_ignore_robots(self, start, goal):
        if self.map_info is None or self.map_data is None:
            return []

        width = self.map_info.width
        height = self.map_info.height
        obstacles = set(self.live_fire_positions) | set(self.live_human_positions)
        obstacles.discard(goal)
        obstacles.discard(start)

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0.0
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if self.map_data[ny * width + nx] > 50:
                    continue
                if neighbor in obstacles:
                    continue
                tentative_g = g_score[current] + 1.0
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.euclidean(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return []

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------
    def direction_between(self, src, dst):
        dx = dst[0] - src[0]
        dy = dst[1] - src[1]
        if dx == 1:
            return 'E'
        if dx == -1:
            return 'W'
        if dy == 1:
            return 'S'
        if dy == -1:
            return 'N'
        return None

    def sync_robot_pos(self, robot_id):
        """Read the actual position from the live entity data published
        by entity_sim, so the solver never drifts from ground truth."""
        for _ in range(10):
            robots = self.live_robot_positions
            if robot_id < len(robots):
                with self.lock:
                    self.robot_pos[robot_id] = robots[robot_id]
                return
            time.sleep(0.05)

    def move_one_step(self, robot_id, direction):
        msg = String()
        msg.data = direction
        self.move_pubs[robot_id].publish(msg)

        with self.lock:
            self.robot_dir[robot_id] = direction

        # Wait for entity_sim to process the move and publish updated entities
        time.sleep(0.35)

        # Sync from ground truth instead of optimistic update
        self.sync_robot_pos(robot_id)

    def navigate_to(self, robot_id, goal):
        """Move robot towards goal one step at a time, re-running A* each
        step.  If the path is blocked, wait and retry.  Detects when the
        robot is truly stuck (position unchanged after move) and recovers."""
        max_steps = 400
        path_fail_count = 0
        stuck_count = 0

        for _ in range(max_steps):
            self.sync_robot_pos(robot_id)
            current = self.robot_pos[robot_id]
            if current == goal:
                return True

            # Try A* with full obstacle avoidance
            path = self.a_star(current, goal, robot_id)

            if not path:
                path_fail_count += 1
                if path_fail_count > 20:
                    # Fallback: ignore robots in pathfinding
                    path = self.a_star_ignore_robots(current, goal)
                    if not path:
                        self.get_logger().warn(
                            f'Robot {robot_id}: truly no path to {goal}')
                        return False
                else:
                    time.sleep(0.5)
                    continue

            path_fail_count = 0
            next_cell = path[0]
            d = self.direction_between(current, next_cell)
            if d is None:
                return False

            # Check if the next cell is occupied by another robot right now
            with self.lock:
                blocked = any(
                    self.robot_pos[i] == next_cell
                    for i in range(4) if i != robot_id)
            if blocked:
                time.sleep(0.4)
                continue

            # Attempt the move
            self.move_one_step(robot_id, d)

            # Check if we actually moved (entity_sim may have rejected it)
            new_pos = self.robot_pos[robot_id]
            if new_pos == current:
                stuck_count += 1
                if stuck_count > 10:
                    self.get_logger().warn(
                        f'Robot {robot_id}: stuck at {current}, giving up on {goal}')
                    return False
                time.sleep(0.3)
            else:
                stuck_count = 0

        self.get_logger().warn(f'Robot {robot_id}: max steps reaching {goal}')
        return False

    # ------------------------------------------------------------------
    # Pick / Drop
    # ------------------------------------------------------------------
    def pick(self, robot_id):
        if self.pick_clients[robot_id].wait_for_service(timeout_sec=2.0):
            future = self.pick_clients[robot_id].call_async(Trigger.Request())
            time.sleep(0.5)
            return future
        return None

    def drop(self, robot_id):
        if self.drop_clients[robot_id].wait_for_service(timeout_sec=2.0):
            future = self.drop_clients[robot_id].call_async(Trigger.Request())
            time.sleep(0.5)
            return future
        return None

    # ------------------------------------------------------------------
    # Solver
    # ------------------------------------------------------------------
    def solve(self):
        if len(self.live_robot_positions) != 4:
            self.get_logger().warn('Not enough robots')
            return

        w = self.map_info.width
        h = self.map_info.height
        half_w = w // 2
        half_h = h // 2

        zones = [
            (0, 0, half_w, half_h),      # 0: top-left
            (half_w, 0, w, half_h),      # 1: top-right
            (0, half_h, half_w, h),      # 2: bottom-left
            (half_w, half_h, w, h),      # 3: bottom-right
        ]

        all_humans = list(self.live_human_positions)
        robot_positions = list(self.live_robot_positions)

        # Group humans by quadrant and compute centroid per quadrant
        quadrant_humans = {}   # qi -> list of (x,y)
        quadrant_centroids = {}  # qi -> (cx, cy)

        for qi, (x0, y0, x1, y1) in enumerate(zones):
            q_humans = [hp for hp in all_humans
                        if x0 <= hp[0] < x1 and y0 <= hp[1] < y1]
            if not q_humans:
                self.get_logger().info(f'Quadrant {qi} has no humans')
                continue
            cx = sum(hp[0] for hp in q_humans) / len(q_humans)
            cy = sum(hp[1] for hp in q_humans) / len(q_humans)
            quadrant_humans[qi] = q_humans
            quadrant_centroids[qi] = (cx, cy)

        # Build all (distance, quadrant, robot) pairs, sort by distance,
        # and greedily assign so each quadrant gets the globally nearest
        # available robot.
        pairs = []
        for qi, centroid in quadrant_centroids.items():
            for rid, rpos in enumerate(robot_positions):
                pairs.append((self.euclidean(rpos, centroid), qi, rid))
        pairs.sort()

        assigned_robots = set()
        assigned_quadrants = set()
        quadrant_assignments = [[] for _ in range(4)]  # indexed by robot id

        for dist, qi, rid in pairs:
            if qi in assigned_quadrants or rid in assigned_robots:
                continue
            assigned_robots.add(rid)
            assigned_quadrants.add(qi)
            quadrant_assignments[rid] = quadrant_humans[qi]
            cx, cy = quadrant_centroids[qi]
            self.get_logger().info(
                f'Quadrant {qi} (centroid ~({cx:.1f},{cy:.1f})) -> Robot {rid} '
                f'(dist={dist:.1f}), {len(quadrant_humans[qi])} humans')

        for rid in range(4):
            if rid not in assigned_robots:
                self.get_logger().info(f'Robot {rid} has no assigned quadrant')

        # Launch worker for each robot (some may have 0 humans)
        threads = []
        for robot_id in range(4):
            t = threading.Thread(
                target=self.robot_worker,
                args=(robot_id, quadrant_assignments[robot_id]),
                daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        self.get_logger().info('All robots finished. Solver complete.')

    def robot_worker(self, robot_id, humans):
        remaining = list(humans)

        while remaining:
            current = self.robot_pos[robot_id]

            # Pick closest human by Euclidean distance
            target = min(remaining, key=lambda h: self.euclidean(current, h))
            self.get_logger().info(
                f'Robot {robot_id} targeting human at {target}')

            # Find all walkable adjacent cells of the human, sorted by
            # distance to the robot so we try the closest first
            adj_cells = []
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                ax, ay = target[0] + dx, target[1] + dy
                if not self.is_wall(ax, ay):
                    adj_cells.append((ax, ay))
            adj_cells.sort(key=lambda c: self.euclidean(current, c))

            if not adj_cells:
                self.get_logger().warn(
                    f'Robot {robot_id}: human at {target} fully walled in')
                remaining.remove(target)
                continue

            # Try each adjacent cell until one works
            reached = False
            for adj_goal in adj_cells:
                if self.navigate_to(robot_id, adj_goal):
                    reached = True
                    break

            if not reached:
                self.get_logger().warn(
                    f'Robot {robot_id}: cannot reach any adj cell of {target}')
                remaining.remove(target)
                continue

            self.pick(robot_id)
            remaining.remove(target)

            # Carry to nearest free edge cell
            edge_cells = self.get_edge_cells()
            current = self.robot_pos[robot_id]
            with self.lock:
                occupied = set(
                    p for i, p in enumerate(self.robot_pos)
                    if i != robot_id and p is not None)
            occupied |= set(self.live_fire_positions)
            occupied |= set(self.live_human_positions)
            free_edges = [c for c in edge_cells if c not in occupied]

            if not free_edges:
                self.get_logger().warn(
                    f'Robot {robot_id}: no free edge cell for drop')
                continue

            # Sort by distance and try each until one is reachable
            free_edges.sort(key=lambda e: self.euclidean(current, e))

            dropped = False
            for drop_target in free_edges[:5]:  # try up to 5 nearest
                if self.navigate_to(robot_id, drop_target):
                    self.drop(robot_id)
                    self.get_logger().info(
                        f'Robot {robot_id} dropped person at edge {drop_target}')
                    dropped = True
                    break

            if not dropped:
                self.get_logger().warn(
                    f'Robot {robot_id}: could not reach any edge for drop')


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
