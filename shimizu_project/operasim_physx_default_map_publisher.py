#!/usr/bin/env python

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import cv2
import numpy as np

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
       
        self.scale_factor = 0.5

        self.declare_parameter('image_file', "seikei_like_project_unity")
        self.image_file = self.get_parameter('image_file').get_parameter_value().string_value

        self.declare_parameter('origin_x', 0.0)
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        
        self.declare_parameter('origin_y', 0.0)
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value

        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.seikei_like_project_unity_dir = get_package_share_directory(self.image_file)
        self.operasim_physx_default_fileld_img_file = os.path.join(self.seikei_like_project_unity_dir, "images", "operasim_physx_default_map.png")

    def timer_callback(self):
        cv_image = cv2.imread(self.operasim_physx_default_fileld_img_file, cv2.IMREAD_GRAYSCALE)
        if cv_image is not None:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            cv_image = cv2.flip(cv_image, 0)
            width = int(cv_image.shape[1] * self.scale_factor)
            height = int(cv_image.shape[0] * self.scale_factor)
            cv_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_AREA)
            occupancy_grid_msg = self.convert_image_to_occupancy_grid(cv_image)
            self.occupancy_grid_pub.publish(occupancy_grid_msg)
        else:
            self.get_logger().error(f"Could not read image file: {self.operasim_physx_default_fileld_img_file}")

    def convert_image_to_occupancy_grid(self, cv_image):
        height, width = cv_image.shape 
        occupancy_grid = OccupancyGrid()

        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"

        occupancy_grid.info.resolution = 0.1 
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = self.origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Normalize image to range [0, 100]
        normalized_image = (cv_image / 255.0 * 100).astype(np.int8)
        # Flatten the image to a single array
        flattened_image = normalized_image.flatten().tolist()
        occupancy_grid.data = flattened_image

        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
