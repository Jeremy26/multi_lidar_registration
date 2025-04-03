#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class LidarStacker(Node):
    def __init__(self):
        super().__init__('lidar_stacker_node')

        # Create subscribers for both point clouds
        velodyne_sub = Subscriber(self, PointCloud2, '/velodyne_points')
        os1_sub = Subscriber(self, PointCloud2, '/os1_cloud_node/points')

        # Synchronize the messages with a time slop of 0.1 sec
        self.ts = ApproximateTimeSynchronizer([velodyne_sub, os1_sub], queue_size=20, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Publisher for the merged point cloud
        self.stacked_pub = self.create_publisher(PointCloud2, '/uncalibrated_points', 5)
        self.get_logger().info("LiDAR stacker node initialized")

    def process_pc(self, msg, color):
        # Read points from the PointCloud2 message
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if points.size == 0:
            self.get_logger().warn("Empty point cloud received.")
            return o3d.geometry.PointCloud()

        pcd = o3d.geometry.PointCloud()
        if points.ndim == 1:
            points_np = np.array([points['x'], points['y'], points['z']]).T.astype(np.float64)
        else:
            points_np = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float64)
        pcd.points = o3d.utility.Vector3dVector(points_np)
        pcd.colors = o3d.utility.Vector3dVector(np.tile(color, (points.shape[0], 1)))
        return pcd

    def pack_rgb(self, colors):
        colors_uint8 = (colors * 255).astype(np.uint32)
        rgb_int = (colors_uint8[:, 0] << 16) | (colors_uint8[:, 1] << 8) | (colors_uint8[:, 2])
        rgb_float = rgb_int.view(np.float32)
        return rgb_float.reshape(-1, 1)

    def sync_callback(self, velodyne_msg, os1_msg):
        self.get_logger().info("Received synchronized messages")
        velodyne_pcd = self.process_pc(velodyne_msg, [0, 1, 0])  # green
        os1_pcd = self.process_pc(os1_msg, [0, 0, 1])              # blue

        # For uncalibrated stacking, simply concatenate the clouds
        merged_pcd = velodyne_pcd + os1_pcd
        merged_pcd_down = merged_pcd.voxel_down_sample(0.02)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "velodyne"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        merged_points = np.asarray(merged_pcd_down.points)
        merged_colors = np.asarray(merged_pcd_down.colors)
        rgb_field = self.pack_rgb(merged_colors)
        merged_data = np.hstack([merged_points, rgb_field])

        merged_msg = pc2.create_cloud(header, fields, merged_data)
        self.stacked_pub.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarStacker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
