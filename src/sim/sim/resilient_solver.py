#!/usr/bin/env python3
"""
Resilient solver.

Phase 1 (no destructions yet):
    Same quadrant strategy as the naive solver --- split map into 4
    quadrants, compute the centroid of humans in each quadrant, and
    greedily assign the nearest robot to each quadrant.

Phase 2 (triggered whenever a robot is destroyed):
    All alive robots abandon their current quadrant plan and switch to
    a growing-circle strategy:

      1. Uncarried humans are collected.
      2. Each alive robot has a circle centred on its current position.
         All circles grow in lockstep (same radius) until every human
         is covered by at least one circle.
      3. A human covered by multiple circles is given to the robot
         that currently has the fewest assigned humans (ties broken
         by Euclidean distance).
      4. Every alive robot replaces its pending queue with the new
         assignment list.

Re-planning is re-run every time another robot is lost, so the surviving
robots keep redistributing the remaining rescue load among themselves.
"""

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


NUM_ROBOTS = 4


class ResilientSolver(Node):
    def __init__(self):
        super().__init__('resilient_solver')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.entities_sub = self.create_subscription(
            MarkerArray, '/entities', self.entities_callback, 10)
        self.move_pubs = [
            self.create_publisher(String, f'/robot{i}/move', 10)
            for i in range(NUM_ROBOTS)]
        self.pick_clients = [
            self.create_client(Trigger, f'/robot{i}/pick')
            for i in range(NUM_ROBOTS)]
        self.drop_clients = [
            self.create_client(Trigger, f'/robot{i}/drop')
            for i in range(NUM_ROBOTS)]

        self.map_data = None
        self.map_info = None

        # Live state from entity_sim
        self.live_human_positions = []
        self.live_fire_positions = []
        self.live_robot_positions = []
        self.live_robot_destroyed = [False] * NUM_ROBOTS

        # Internal tracked positions / headings used by workers
        self.robot_pos = [None] * NUM_ROBOTS
        self.robot_dir = [None] * NUM_ROBOTS

        # Per-robot assignment queue and control flags
        self.assignments = [[] for _ in range(NUM_ROBOTS)]  # list of humans
        self.carrying = [False] * NUM_ROBOTS
        # Version counter incremented on every re-plan; workers check it
        # to abandon their current target mid-way.
        self.plan_version = 0
        self.prev_destroyed = [False] * NUM_ROBOTS

        self.lock = threading.Lock()
        self.started = False

        self.get_logger().info(
            'Resilient solver started. Waiting for map and entities...')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def map_callback(self, msg):
        self.map_data = list(msg.data)
        self.map_info = msg.info
        self.get_logger().info('Map received by resilient solver')

    def entities_callback(self, msg):
        humans, fires, robots = [], [], []
        destroyed = [False] * NUM_ROBOTS

        for marker in msg.markers:
            x = int(marker.pose.position.x / 0.05)
            y = int(marker.pose.position.y / 0.05)
            if marker.ns == 'humans':
                humans.append((x, y))
            elif marker.ns == 'fires':
                fires.append((x, y))
            elif marker.ns == 'robots':
                rid = marker.id
                # pad list if needed
                while len(robots) <= rid:
                    robots.append(None)
                robots[rid] = (x, y)
                # gray marker -> destroyed
                if (abs(marker.color.r - 0.5) < 1e-3 and
                        abs(marker.color.g - 0.5) < 1e-3 and
                        abs(marker.color.b - 0.5) < 1e-3):
                    destroyed[rid] = True

        # Normalise robots to length NUM_ROBOTS
        while len(robots) < NUM_ROBOTS:
            robots.append(None)

        self.live_human_positions = humans
        self.live_fire_positions = fires
        self.live_robot_positions = robots
        self.live_robot_destroyed = destroyed

        # Boot the solver once everything is available
        if (not self.started and self.map_info is not None
                and all(r is not None for r in robots)):
            self.started = True
            with self.lock:
                for i in range(NUM_ROBOTS):
                    self.robot_pos[i] = robots[i]
                    self.robot_dir[i] = None
            self.get_logger().info(
                'All data received. Launching resilient solver...')
            t = threading.Thread(target=self.solve, daemon=True)
            t.start()
            # Start the destruction monitor
            m = threading.Thread(target=self.monitor_destruction, daemon=True)
            m.start()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def euclidean(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

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

    def alive_robot_ids(self):
        return [i for i in range(NUM_ROBOTS)
                if not self.live_robot_destroyed[i]]

    # ------------------------------------------------------------------
    # A* (same as naive solver: dynamic robot obstacles)
    # ------------------------------------------------------------------
    def a_star(self, start, goal, robot_id, ignore_robots=False):
        if self.map_info is None or self.map_data is None:
            return []

        width = self.map_info.width
        height = self.map_info.height

        dynamic = set()
        if not ignore_robots:
            with self.lock:
                for i in range(NUM_ROBOTS):
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

        # Destroyed robots are permanent obstacles
        for i in range(NUM_ROBOTS):
            if self.live_robot_destroyed[i] and i != robot_id:
                pos = self.live_robot_positions[i]
                if pos is not None:
                    dynamic.add(pos)

        obstacles = (set(self.live_fire_positions)
                     | set(self.live_human_positions)
                     | dynamic)
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
    @staticmethod
    def direction_between(src, dst):
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
        for _ in range(10):
            robots = self.live_robot_positions
            if robot_id < len(robots) and robots[robot_id] is not None:
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

        time.sleep(0.35)
        self.sync_robot_pos(robot_id)

    def navigate_to(self, robot_id, goal, plan_version_at_start):
        """Move robot one step at a time toward goal, re-running A* each
        step.  Aborts if the plan version changes (re-plan triggered) or
        if the robot itself is destroyed."""
        max_steps = 400
        path_fail_count = 0
        stuck_count = 0

        for _ in range(max_steps):
            if self.plan_version != plan_version_at_start:
                return False  # pre-empted by re-plan
            if self.live_robot_destroyed[robot_id]:
                return False

            self.sync_robot_pos(robot_id)
            current = self.robot_pos[robot_id]
            if current == goal:
                return True

            path = self.a_star(current, goal, robot_id)
            if not path:
                path_fail_count += 1
                if path_fail_count > 20:
                    path = self.a_star(current, goal, robot_id,
                                       ignore_robots=True)
                    if not path:
                        self.get_logger().warn(
                            f'Robot {robot_id}: no path to {goal}')
                        return False
                else:
                    time.sleep(0.5)
                    continue

            path_fail_count = 0
            next_cell = path[0]
            d = self.direction_between(current, next_cell)
            if d is None:
                return False

            with self.lock:
                blocked = any(
                    self.robot_pos[i] == next_cell
                    for i in range(NUM_ROBOTS) if i != robot_id)
            if blocked:
                time.sleep(0.4)
                continue

            self.move_one_step(robot_id, d)

            new_pos = self.robot_pos[robot_id]
            if new_pos == current:
                stuck_count += 1
                if stuck_count > 10:
                    self.get_logger().warn(
                        f'Robot {robot_id}: stuck at {current}, '
                        f'giving up on {goal}')
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
            self.carrying[robot_id] = True
            return future
        return None

    def drop(self, robot_id):
        if self.drop_clients[robot_id].wait_for_service(timeout_sec=2.0):
            future = self.drop_clients[robot_id].call_async(Trigger.Request())
            time.sleep(0.5)
            self.carrying[robot_id] = False
            return future
        return None

    # ------------------------------------------------------------------
    # Planning strategies
    # ------------------------------------------------------------------
    def compute_quadrant_assignments(self):
        """Phase 1: 4-quadrant centroid assignment."""
        w = self.map_info.width
        h = self.map_info.height
        half_w = w // 2
        half_h = h // 2

        zones = [
            (0, 0, half_w, half_h),
            (half_w, 0, w, half_h),
            (0, half_h, half_w, h),
            (half_w, half_h, w, h),
        ]

        all_humans = list(self.live_human_positions)
        robot_positions = list(self.live_robot_positions)

        quadrant_humans = {}
        quadrant_centroids = {}
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

        pairs = []
        for qi, centroid in quadrant_centroids.items():
            for rid, rpos in enumerate(robot_positions):
                if rpos is None:
                    continue
                pairs.append((self.euclidean(rpos, centroid), qi, rid))
        pairs.sort()

        assigned_robots = set()
        assigned_quadrants = set()
        assignments = [[] for _ in range(NUM_ROBOTS)]

        for dist, qi, rid in pairs:
            if qi in assigned_quadrants or rid in assigned_robots:
                continue
            assigned_robots.add(rid)
            assigned_quadrants.add(qi)
            assignments[rid] = list(quadrant_humans[qi])
            cx, cy = quadrant_centroids[qi]
            self.get_logger().info(
                f'[QUADRANT] Q{qi} centroid ({cx:.1f},{cy:.1f}) '
                f'-> Robot {rid} ({len(quadrant_humans[qi])} humans)')

        return assignments

    def compute_circle_assignments(self, remaining_humans):
        """Phase 2: grow circles around each alive robot until every
        remaining human is covered, then resolve overlaps by giving
        disputed humans to the least-loaded robot."""
        alive = self.alive_robot_ids()
        assignments = [[] for _ in range(NUM_ROBOTS)]

        if not alive or not remaining_humans:
            return assignments

        robot_pts = {rid: self.live_robot_positions[rid] for rid in alive
                     if self.live_robot_positions[rid] is not None}
        if not robot_pts:
            return assignments

        # Find the minimum radius at which every human is covered by at
        # least one alive robot's circle.  Equivalently: for each human,
        # the minimum distance to any alive robot; the required radius
        # is the max of those.
        human_nearest = {}
        for h in remaining_humans:
            d_min = float('inf')
            for rid, rp in robot_pts.items():
                d = self.euclidean(rp, h)
                if d < d_min:
                    d_min = d
            human_nearest[h] = d_min

        required_radius = max(human_nearest.values())
        self.get_logger().info(
            f'[CIRCLES] alive={alive}  humans={len(remaining_humans)}  '
            f'radius grown to {required_radius:.2f}')

        # Build candidate (robot, human) sets (humans within radius)
        candidates = {rid: [] for rid in alive}
        for h in remaining_humans:
            for rid, rp in robot_pts.items():
                if self.euclidean(rp, h) <= required_radius + 1e-9:
                    candidates[rid].append(h)

        # Resolve overlaps: process humans in some deterministic order,
        # assign each to the currently least-loaded candidate robot
        # (ties broken by distance).
        load = {rid: 0 for rid in alive}
        assignment_per_human = {}

        # Order humans by fewest candidates first (hardest to place),
        # then by distance to their nearest alive robot.
        def human_key(h):
            cands = [rid for rid in alive
                     if self.euclidean(robot_pts[rid], h)
                     <= required_radius + 1e-9]
            return (len(cands), human_nearest[h])

        for h in sorted(remaining_humans, key=human_key):
            cands = [rid for rid in alive
                     if self.euclidean(robot_pts[rid], h)
                     <= required_radius + 1e-9]
            if not cands:
                # Shouldn't happen given how radius was computed
                cands = list(alive)
            # Pick least-loaded, tie-break by distance
            cands.sort(key=lambda r: (load[r],
                                      self.euclidean(robot_pts[r], h)))
            chosen = cands[0]
            assignment_per_human[h] = chosen
            load[chosen] += 1

        for h, rid in assignment_per_human.items():
            assignments[rid].append(h)

        for rid in alive:
            self.get_logger().info(
                f'[CIRCLES] Robot {rid} gets {len(assignments[rid])} humans')

        return assignments

    def replan(self, reason):
        """Recompute assignments for every alive robot and bump the plan
        version so in-flight workers abandon their current goal."""
        with self.lock:
            # Humans still on the map (not yet picked)
            remaining = list(self.live_human_positions)
            new_assignments = self.compute_circle_assignments(remaining)
            for rid in range(NUM_ROBOTS):
                if self.live_robot_destroyed[rid]:
                    self.assignments[rid] = []
                else:
                    self.assignments[rid] = new_assignments[rid]
            self.plan_version += 1
            version = self.plan_version
        self.get_logger().warn(
            f'[REPLAN #{version}] {reason}.  Switched to circle strategy.')

    # ------------------------------------------------------------------
    # Destruction monitor
    # ------------------------------------------------------------------
    def monitor_destruction(self):
        """Polls live_robot_destroyed; whenever a new robot dies, fires
        a re-plan."""
        while rclpy.ok():
            time.sleep(0.2)
            current = list(self.live_robot_destroyed)
            newly_dead = [i for i in range(NUM_ROBOTS)
                          if current[i] and not self.prev_destroyed[i]]
            if newly_dead:
                self.prev_destroyed = current
                self.replan(
                    f'robot {newly_dead} destroyed '
                    f'(alive={self.alive_robot_ids()})')
            else:
                self.prev_destroyed = current

    # ------------------------------------------------------------------
    # Solver entry point
    # ------------------------------------------------------------------
    def solve(self):
        if not all(r is not None for r in self.live_robot_positions):
            self.get_logger().warn('Not all robots present')
            return

        # Phase 1: quadrant assignment
        with self.lock:
            self.assignments = self.compute_quadrant_assignments()
            self.plan_version = 1

        threads = []
        for rid in range(NUM_ROBOTS):
            t = threading.Thread(
                target=self.robot_worker, args=(rid,), daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        self.get_logger().info(
            'All robots finished (or destroyed).  Resilient solver complete.')

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------
    def robot_worker(self, robot_id):
        while rclpy.ok():
            if self.live_robot_destroyed[robot_id]:
                self.get_logger().warn(
                    f'Robot {robot_id} destroyed. Worker exiting.')
                return

            # Snapshot assignment at this plan version
            with self.lock:
                version = self.plan_version
                my_humans = list(self.assignments[robot_id])

            if not my_humans:
                # Maybe a re-plan will hand us humans later; wait briefly
                # unless the simulation is effectively over.
                time.sleep(0.5)
                with self.lock:
                    new_version = self.plan_version
                    new_humans = list(self.assignments[robot_id])
                if new_version == version and not new_humans:
                    # Nothing to do and no new plan -> exit worker
                    return
                continue

            self.sync_robot_pos(robot_id)
            current = self.robot_pos[robot_id]

            # Filter out humans that have already been rescued (no longer
            # in live_human_positions).  entity_sim removes them on pick.
            live = set(self.live_human_positions)
            my_humans = [h for h in my_humans if h in live]
            if not my_humans:
                with self.lock:
                    self.assignments[robot_id] = []
                continue

            # Go to nearest human
            target = min(my_humans, key=lambda h: self.euclidean(current, h))
            self.get_logger().info(
                f'Robot {robot_id} targeting human at {target} '
                f'(plan v{version})')

            # Find walkable adjacent cells
            adj_cells = []
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                ax, ay = target[0] + dx, target[1] + dy
                if not self.is_wall(ax, ay):
                    adj_cells.append((ax, ay))
            adj_cells.sort(key=lambda c: self.euclidean(current, c))

            if not adj_cells:
                self.get_logger().warn(
                    f'Robot {robot_id}: human at {target} walled in')
                with self.lock:
                    if target in self.assignments[robot_id]:
                        self.assignments[robot_id].remove(target)
                continue

            reached = False
            for adj_goal in adj_cells:
                if self.plan_version != version:
                    break
                if self.navigate_to(robot_id, adj_goal, version):
                    reached = True
                    break

            if self.plan_version != version:
                # Re-plan happened; restart loop with new assignments
                continue
            if self.live_robot_destroyed[robot_id]:
                return

            if not reached:
                self.get_logger().warn(
                    f'Robot {robot_id}: cannot reach {target}')
                with self.lock:
                    if target in self.assignments[robot_id]:
                        self.assignments[robot_id].remove(target)
                continue

            self.pick(robot_id)
            with self.lock:
                if target in self.assignments[robot_id]:
                    self.assignments[robot_id].remove(target)

            # Head for a free edge cell to drop
            edge_cells = self.get_edge_cells()
            self.sync_robot_pos(robot_id)
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

            free_edges.sort(key=lambda e: self.euclidean(current, e))

            dropped = False
            for drop_target in free_edges[:5]:
                if self.live_robot_destroyed[robot_id]:
                    return
                # NOTE: we deliberately keep navigating to an edge even
                # if a re-plan happens; dropping the carried human is
                # always the right thing to do.
                if self.navigate_to(robot_id, drop_target, self.plan_version):
                    self.drop(robot_id)
                    self.get_logger().info(
                        f'Robot {robot_id} dropped at edge {drop_target}')
                    dropped = True
                    break

            if not dropped:
                self.get_logger().warn(
                    f'Robot {robot_id}: could not reach any edge for drop')


def main(args=None):
    rclpy.init(args=args)
    node = ResilientSolver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
