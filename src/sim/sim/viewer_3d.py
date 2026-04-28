#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import pyvista as pv
import numpy as np
import threading
from collections import defaultdict


class Viewer3D(Node):
    def __init__(self):
        super().__init__('viewer_3d')
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_entities = self.create_subscription(
            MarkerArray, '/entities', self.entities_callback, 10)

        self.map_data = None
        self.map_info = None
        self.entities_data = None
        self.entities_lock = threading.Lock()
        self.running = True
        self.entity_actors = {}  # (ns, id) -> actor

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def entities_callback(self, msg):
        with self.entities_lock:
            self.entities_data = msg.markers

    def build_map(self):
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        height, width = self.map_data.shape

        walls = []
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] > 50:
                    px = origin_x + x * resolution
                    py = origin_y + y * resolution
                    walls.append(pv.Box(bounds=[
                        px, px + resolution,
                        py, py + resolution,
                        0, resolution * 3
                    ]))

        if walls:
            combined = walls[0]
            for w in walls[1:]:
                combined = combined + w
            return combined
        return None

    def _make_entity_mesh(self, ns, entity_w, entity_h):
        """Create a canonical mesh centred at the origin for an entity type."""
        if ns == 'humans':
            return pv.Cylinder(
                center=(0.0, 0.0, entity_h / 2),
                radius=entity_w / 2,
                height=entity_h,
                direction=(0, 0, 1)
            )
        elif ns == 'fires':
            return pv.Sphere(
                center=(0.0, 0.0, entity_w / 2),
                radius=entity_w / 2
            )
        elif ns == 'robots':
            return pv.Box(bounds=[
                -entity_w / 2, entity_w / 2,
                -entity_w / 2, entity_w / 2,
                0, entity_h
            ])
        return None

    def update_entities(self, markers):
        cell = self.map_info.resolution
        entity_w = cell * 0.6
        entity_h = cell * 1.5

        # Build lookup: (ns, id) -> marker
        incoming = {}
        for marker in markers:
            incoming[(marker.ns, marker.id)] = marker

        # Keys present in the scene right now
        existing_keys = set(self.entity_actors.keys())
        incoming_keys = set(incoming.keys())

        # --- Remove actors that no longer exist ---
        for key in existing_keys - incoming_keys:
            self.plotter.remove_actor(self.entity_actors[key])
            del self.entity_actors[key]

        # --- Add actors for newly appearing entities ---
        for key in incoming_keys - existing_keys:
            marker = incoming[key]
            ns = marker.ns
            color = [
                int(marker.color.r * 255),
                int(marker.color.g * 255),
                int(marker.color.b * 255)
            ]
            mesh = self._make_entity_mesh(ns, entity_w, entity_h)
            if mesh is None:
                continue
            ambient = 0.6 if ns == 'fires' else 0.4
            actor = self.plotter.add_mesh(
                mesh, color=color, ambient=ambient, smooth_shading=True)
            self.entity_actors[key] = actor

        # --- Move existing actors to their new positions (no remove/add) ---
        for key in incoming_keys & existing_keys:
            marker = incoming[key]
            pos = marker.pose.position
            actor = self.entity_actors[key]
            actor.SetPosition(pos.x, pos.y, 0.0)

    def run_viewer(self):
        # Wait for map data from ROS (spun on a separate thread)
        import time
        while self.map_data is None:
            time.sleep(0.05)

        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        height, width = self.map_data.shape
        extent_x = width * resolution
        extent_y = height * resolution

        try:
            self.plotter = pv.Plotter()
        except Exception as e:
            self.get_logger().error(f'Failed to create plotter: {e}')
            return

        self.plotter.set_background('#1a1a1a')
        self.plotter.enable_terrain_style()

        walls_mesh = self.build_map()
        if walls_mesh:
            self.plotter.add_mesh(walls_mesh, color='#4a4a4a', show_scalar_bar=False, opacity=0.95)

        floor = pv.Plane(
            center=(extent_x / 2, extent_y / 2, -0.001),
            direction=(0, 0, 1),
            i_size=extent_x,
            j_size=extent_y
        )
        self.plotter.add_mesh(floor, color='#2a2a2a', show_edges=False)

        self.plotter.reset_camera()
        self.plotter.camera_position = 'iso'

        # Render initial entity state if already received
        with self.entities_lock:
            if self.entities_data:
                self.update_entities(self.entities_data)
                self.entities_data = None

        self.get_logger().info('3D viewer window opening...')

        # show() with interactive_update=True returns immediately and lets us
        # drive frames manually via plotter.update()
        self.plotter.show(
            title='3D Viewer',
            auto_close=False,
            interactive_update=True
        )

        while self.running and not self.plotter._closed:
            # Drain any pending entity update from the ROS callback thread
            with self.entities_lock:
                pending = self.entities_data
                if pending is not None:
                    self.entities_data = None

            if pending is not None:
                self.update_entities(pending)
                self.get_logger().info(f'Updated {len(pending)} entity markers')

            self.plotter.update()   # process GUI events + render
            time.sleep(0.05)       # ~20 fps

        self.running = False
        try:
            self.plotter.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Viewer3D()

    # Spin ROS on a background thread so callbacks fire while pyvista owns the main thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_viewer()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
