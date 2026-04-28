#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import pyvista as pv
import numpy as np
from threading import Thread


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
        self.running = True

        thread = Thread(target=self.run_viewer)
        thread.daemon = True
        thread.start()

        self.get_logger().info('3D Viewer started')
        self.timer = self.create_timer(0.1, self.update_data)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.resolution = msg.info.resolution

    def entities_callback(self, msg):
        self.entities_data = msg.markers

    def update_data(self):
        pass

    def run_viewer(self):
        while self.map_data is None:
            import time
            time.sleep(0.1)

        self.plotter = pv.Plotter()
        self.plotter.set_background('black')
        self.plotter.add_axes()
        self.plotter.hide_axes()
        self.plotter.hide_actors()

        resolution = self.map_info.resolution if self.map_info else 0.05
        origin_x = self.map_info.origin.position.x if self.map_info else 0.0
        origin_y = self.map_info.origin.position.y if self.map_info else 0.0

        height, width = self.map_data.shape
        extent_x = width * resolution
        extent_y = height * resolution

        grid = pv.UniformGrid(dimensions=(width, height, 1))
        grid.origin = (origin_x, origin_y, 0)
        grid.spacing = (resolution, resolution, resolution)

        wall_values = np.where(self.map_data > 0, 1, np.nan)
        wall_values = wall_values.flatten(order='F')
        grid.cell_data['walls'] = wall_values

        walls = grid.threshold([0.5, 1.5], invert=True)
        if walls.n_cells > 0:
            self.plotter.add_mesh(walls, color='#505050', show_scalar_bar=False, opacity=0.9)

        floor = pv.Plane(
            center=(extent_x/2, extent_y/2, -0.01),
            direction=(0, 0, 1),
            i_size=extent_x,
            j_size=extent_y
        )
        self.plotter.add_mesh(floor, color='#0a0a0a', show_edges=False)

        self.entities_mesh = []
        self.plotter.reset_camera()
        self.plotter.camera_position = 'iso'
        self.plotter.enable_terrain_style()
        self.plotter.render()

        while self.running:
            if self.entities_data:
                for actor in self.entities_mesh:
                    self.plotter.remove_actor(actor)
                self.entities_mesh = []

                for marker in self.entities_data:
                    pos = marker.pose.position
                    scale = marker.scale
                    color = (marker.color.r, marker.color.g, marker.color.b)

                    if marker.ns == 'humans':
                        mesh = pv.Cylinder(
                            center=(pos.x, pos.y, pos.z),
                            radius=scale.x / 2,
                            height=scale.z,
                            direction=(0, 0, 1)
                        )
                        self.entities_mesh.append(
                            self.plotter.add_mesh(mesh, color=color, ambient=0.3))

                    elif marker.ns == 'fires':
                        mesh = pv.Sphere(
                            center=(pos.x, pos.y, pos.z),
                            radius=scale.x / 2
                        )
                        self.entities_mesh.append(
                            self.plotter.add_mesh(mesh, color=color, ambient=0.5))

                    elif marker.ns == 'robots':
                        mesh = pv.Box(
                            bounds=[
                                pos.x - scale.x/2, pos.x + scale.x/2,
                                pos.y - scale.y/2, pos.y + scale.y/2,
                                pos.z - scale.z/2, pos.z + scale.z/2
                            ]
                        )
                        self.entities_mesh.append(
                            self.plotter.add_mesh(mesh, color=color, ambient=0.3))

                self.entities_data = None
                self.plotter.render()

            import time
            time.sleep(0.1)

        self.plotter.close()


def main(args=None):
    rclpy.init(args=args)
    node = Viewer3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()