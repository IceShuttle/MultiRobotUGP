#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String as StringMsg
from std_srvs.srv import Trigger

import random

class EntitySim(Node):
    def __init__(self):
        super().__init__('entity_sim')
        self.declare_parameter('num_humans', 5)
        self.num_humans = self.get_parameter('num_humans').value
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.entity_pub = self.create_publisher(MarkerArray, '/entities', 10)
        self.log_pub = self.create_publisher(StringMsg, '/action_log', 10)
        self.timer = self.create_timer(2.0, self.publish_entities)
        self.map_info = None
        self.free_cells = []
        self.human_positions = []
        self.fire_positions = []
        self.robot_positions = []
        self.carrying = [False] * 4
        self.destroyed = [False] * 4
        self.get_logger().info(f'EntitySim started with {self.num_humans} humans. Waiting for map...')

    def map_callback(self, msg):
        if self.map_info is not None:
            return  # only process first map
        self.map_info = msg.info
        self.map_data = list(msg.data)  # copy
        width = msg.info.width
        height = msg.info.height

        self.free_cells = []
        for y in range(height):
            for x in range(width):
                if self.map_data[y * width + x] <= 0:  # free
                    self.free_cells.append((x, y))

        # Separate edge cells for robots
        edge_cells = [(x, y) for x, y in self.free_cells if x == 0 or x == width-1 or y == 0 or y == height-1]
        non_edge_cells = [(x, y) for x, y in self.free_cells if (x, y) not in edge_cells]

        # Scatter humans and fires in non-edge
        random.shuffle(non_edge_cells)
        self.human_positions = non_edge_cells[:self.num_humans]
        self.fire_positions = non_edge_cells[self.num_humans:self.num_humans+3]   # 3 fires

        # Robots at edges (exclude fires), or non-edge if not enough
        edge_cells = [cell for cell in edge_cells if cell not in self.fire_positions]
        random.shuffle(edge_cells)
        self.robot_positions = edge_cells[:4]
        if len(self.robot_positions) < 4:
            needed = 4 - len(self.robot_positions)
            non_edge_cells = [cell for cell in non_edge_cells if cell not in self.fire_positions]
            self.robot_positions += non_edge_cells[8:8+needed]

        self.get_logger().info(f'Placed {len(self.human_positions)} humans, {len(self.fire_positions)} fires, {len(self.robot_positions)} robots on free cells')

        # Create subscribers and services for each robot
        for robot_id in range(4):
            self.create_subscription(
                StringMsg, f'/robot{robot_id}/move',
                lambda msg, rid=robot_id: self.move_callback(msg, rid), 10)
            self.create_service(
                Trigger, f'/robot{robot_id}/pick',
                lambda req, res, rid=robot_id: self.pick_callback(req, res, rid))
            self.create_service(
                Trigger, f'/robot{robot_id}/remove_fire',
                lambda req, res, rid=robot_id: self.remove_fire_callback(req, res, rid))
            self.create_service(
                Trigger, f'/robot{robot_id}/drop',
                lambda req, res, rid=robot_id: self.drop_callback(req, res, rid))
            self.create_service(
                Trigger, f'/robot{robot_id}/destroy',
                lambda req, res, rid=robot_id: self.destroy_callback(req, res, rid))

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

        # Robots - green cubes (gray if destroyed)
        for i, (x, y) in enumerate(self.robot_positions):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = timestamp
            marker.ns = 'robots'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x * 0.05 + 0.025
            marker.pose.position.y = y * 0.05 + 0.025
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.6
            if self.destroyed[i]:
                marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.9)  # gray
            else:
                marker.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9)  # green
            markers.markers.append(marker)

        self.entity_pub.publish(markers)

    def move_callback(self, msg, id):
        if self.map_info is None or self.map_data is None or self.destroyed[id]:
            return
        direction = msg.data.upper()
        if id >= len(self.robot_positions):
            return
        x, y = self.robot_positions[id]
        dx, dy = 0, 0
        if direction == 'N':
            dy = -1
        elif direction == 'S':
            dy = 1
        elif direction == 'E':
            dx = 1
        elif direction == 'W':
            dx = -1
        else:
            return

        nx, ny = x + dx, y + dy
        if not (0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height):
            return
        idx = ny * self.map_info.width + nx
        occupied = (self.map_data[idx] > 50 or 
                   (nx, ny) in self.robot_positions or 
                   (nx, ny) in self.fire_positions or 
                   (nx, ny) in self.human_positions)
        if not occupied:
            self.robot_positions[id] = (nx, ny)
            self.publish_entities()

    def pick_callback(self, req, res, id):
        res.success = False
        if self.destroyed[id] or id >= len(self.robot_positions) or self.carrying[id]:
            return res
        rx, ry = self.robot_positions[id]
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = rx + dx, ry + dy
            if (nx, ny) in self.human_positions:
                self.human_positions.remove((nx, ny))
                self.carrying[id] = True
                res.success = True
                self.log_pub.publish(StringMsg(data=f'Robot {id} picked up a person'))
                break
        self.publish_entities()
        return res

    def remove_fire_callback(self, req, res, id):
        res.success = False
        if self.destroyed[id] or id >= len(self.robot_positions):
            return res
        pos = self.robot_positions[id]
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = pos[0] + dx, pos[1] + dy
            if (nx, ny) in self.fire_positions:
                self.fire_positions.remove((nx, ny))
                res.success = True
                self.log_pub.publish(StringMsg(data=f'Robot {id} extinguished fire'))
                break
        self.publish_entities()
        return res

    def drop_callback(self, req, res, id):
        res.success = False
        if self.destroyed[id] or id >= len(self.robot_positions) or not self.carrying[id]:
            return res
        pos = self.robot_positions[id]
        idx = pos[1] * self.map_info.width + pos[0]
        is_edge = (pos[0] == 0 or pos[0] == self.map_info.width-1 or pos[1] == 0 or pos[1] == self.map_info.height-1)
        if not is_edge:
            self.get_logger().warn(f'Robot {id} cannot drop - not at edge cell')
            return res
        if self.map_data[idx] > 0:
            self.get_logger().warn(f'Robot {id} cannot drop - obstacle at edge')
            return res
        if pos in self.human_positions or pos in self.fire_positions:
            self.get_logger().warn(f'Robot {id} cannot drop - occupied at edge')
            return res
        self.carrying[id] = False
        res.success = True
        self.get_logger().info(f'Robot {id} rescued a person at exit {pos}')
        self.log_pub.publish(StringMsg(data=f'Robot {id} rescued a person'))
        self.publish_entities()
        return res

    def destroy_callback(self, req, res, id):
        res.success = False
        if id < len(self.robot_positions) and not self.destroyed[id]:
            self.destroyed[id] = True
            res.success = True
            self.get_logger().info(f'Robot {id} destroyed (now acts as obstacle)')
            self.log_pub.publish(StringMsg(data=f'Robot {id} was destroyed'))
            self.publish_entities()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = EntitySim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
