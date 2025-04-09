#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import tf2_ros
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from transforms3d.quaternions import mat2quat

class LidarCalibrator(Node):
    def __init__(self):
        super().__init__('lidar_calibration_node')

        self.scales = #TODO
        self.thresholds = #TODO
        self.time_slop = 0.1
        self.icp_max_correspondence_distance = 0.05
        self.gicp_max_correspondence_distance = 0.5
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.transform_os1 = None

        self.declare_parameter('use_initial_guess', False)
        use_initial_guess = self.get_parameter('use_initial_guess').value

        if use_initial_guess:
            self.current_transformation = np.array([
                [ 0.73347418, -0.67960607, 0.01229746 ,  0.0592959],
                [ 0.67970715,  0.73324416,  -0.01874016, -0.24557677],
                [0.00371889, 0.02210409, 0.99974876, -0.14750762],
                [0, 0, 0, 1]
            ])      
        else:
            self.current_transformation = #TODO

        
        self.declare_parameter('os1_transform_fixed', False)
        self.os1_transform_fixed = self.get_parameter('os1_transform_fixed').value

        if self.os1_transform_fixed:
            self.final_transform_os1 = np.array([
                [ 0.73347418, -0.67960607, 0.01229746 ,  0.0592959],
                [ 0.67970715,  0.73324416,  -0.01874016, -0.24557677],
                [0.00371889, 0.02210409, 0.99974876, -0.14750762],
                [0, 0, 0, 1]
            ])      
        else:
            self.final_transform_os1 = #TODO

        # Added variables for early stopping
        self.rmse_threshold = #TODO  # Adjust it as needed
        self.transform_os1_counter = 0  # A counter for the number of frames the transform is used
        self.max_iterations = 5  # maximum number of iterations, prevent infinite loops

        velodyne_sub = Subscriber(self, PointCloud2, '/velodyne_points')
        os1_sub = Subscriber(self, PointCloud2, '/os1_cloud_node/points')

        self.ts = ApproximateTimeSynchronizer(
            [velodyne_sub, os1_sub],
            queue_size=20,
            slop=self.time_slop
        )
        self.ts.registerCallback(self.sync_callback)

        self.merged_pub = self.create_publisher(PointCloud2, '/merged_points', 5)
        self.get_logger().info("LiDAR calibration node initialized")

    def process_pc(self, msg, color):
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

    def preprocess_point_cloud(self, pcd, voxel_size):
        if len(pcd.points) == 0:
            self.get_logger().warn("Empty point cloud detected during preprocessing. Skipping...")
            return pcd

        pcd_down = #TODO: Downsample
        #TODO: Estimate Normals
        #TODO: Remove Outliers
        return pcd_down

    def preprocess_point_cloud_fgr(self, pcd, voxel_size):
        self.get_logger().info(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = ##TODO: Downsample
        radius_normal = voxel_size * 2
        self.get_logger().info(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        radius_feature = voxel_size * 5
        self.get_logger().info(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = #TODO: Compute FPFH Features
        return pcd_down, pcd_fpfh

    def fast_global_registration(self, source, target, voxel_size):
        source_down, source_fpfh = self.preprocess_point_cloud_fgr(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud_fgr(target, voxel_size)
        if len(source_down.points) == 0 or len(target_down.points) == 0:
            self.get_logger().warn("Empty point cloud detected in fast global registration.")
            return np.eye(4)
        distance_threshold = voxel_size * 0.5
        result = #TODO: Run FGR
        return result.transformation

    def register_gicp(self, source, target, init_transformation=np.eye(4)):
        try:
            gicp = #TODO: GICP
            return #TODO: 
        except Exception as e:
            self.get_logger().error(f"GICP registration failed: {e}")
            return None

    def register_icp(self, source, target, init_transformation=np.eye(4)):
        icp = #TODO: ICP
        return #TODO

    def multi_scale_registration(self, source, target):
        init_transformation = #TODO: FGR
        current_transformation = init_transformation
        previous_rmse = float('inf')

        for iteration in range(self.max_iterations):
            for voxel_size in self.scales:
                source_down = #TODO: Preprocess
                target_down = #TODO: Preprocess

                if len(source_down.points) == 0 or len(target_down.points) == 0:
                    self.get_logger().warn("Skipping registration due to empty downsampled point cloud.")
                    return current_transformation, previous_rmse  # Return the last valid values

                gicp_transformation = #TODO: GICP

                if gicp_transformation is None:
                    self.get_logger().warn("GICP failed, skipping this scale.")
                    return current_transformation, previous_rmse # Return the last valid values

                icp_result = #TODO: ICP

                current_transformation = #TODO: ICP.Transform
                self.get_logger().info("Multi-scale registration done at voxel size: {}".format(voxel_size))
            
            #calculate the inlier_rmse score from the full point clouds and transformations
            evaluation = o3d.pipelines.registration.evaluate_registration(source, target, self.icp_max_correspondence_distance, current_transformation)

            current_rmse = evaluation.inlier_rmse


            self.get_logger().info(f"Iteration {iteration+1}, RMSE: {current_rmse:.8f}")

            if current_rmse < self.rmse_threshold:
                self.get_logger().info(f"RMSE below threshold. Converged in {iteration+1} iterations.")
                return current_transformation, current_rmse  # return the result object inlier_rmse
           # Check for non-decreasing RMSE to prevent oscillations
            if current_rmse >= previous_rmse:
                 self.get_logger().warn("RMSE not decreasing. Stopping iteration.")
                 return current_transformation, previous_rmse
            previous_rmse = current_rmse

        self.get_logger().warn("Maximum iterations reached. Registration may not have converged.")
        return current_transformation, previous_rmse  # return the result object inlier_rmse

    def publish_transform(self, transformation, source_frame, target_frame):
        if transformation is None:
            self.get_logger().warn(f"Transformation for {source_frame} not available.")
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = target_frame
        tf_msg.child_frame_id = source_frame

        tf_msg.transform.translation.x = float(transformation[0, 3])
        tf_msg.transform.translation.y = float(transformation[1, 3])
        tf_msg.transform.translation.z = float(transformation[2, 3])

        rotation_matrix = transformation[:3, :3]
        q = mat2quat(rotation_matrix)
        tf_msg.transform.rotation.x = float(q[1])
        tf_msg.transform.rotation.y = float(q[2])
        tf_msg.transform.rotation.z = float(q[3])
        tf_msg.transform.rotation.w = float(q[0])

        self.tf_broadcaster.sendTransform(tf_msg)

    def pack_rgb(self, colors):
        colors_uint8 = (colors * 255).astype(np.uint32)
        rgb_int = (colors_uint8[:, 0] << 16) | (colors_uint8[:, 1] << 8) | (colors_uint8[:, 2])
        rgb_float = rgb_int.view(np.float32)
        return rgb_float.reshape(-1, 1)

    def sync_callback(self, velodyne_msg, os1_msg):
        self.get_logger().info("Received synchronized messages")

        velodyne_pcd = #TODO: Process
        os1_pcd = #TODO: Process

        # OS1 Registration with early stopping
        if not self.os1_transform_fixed:
            self.transform_os1, rmse_os1 = #TODO: MultiScale Registration

            if rmse_os1 is not None and rmse_os1 < self.rmse_threshold and rmse_os1 !=0:
                self.get_logger().info(f"OS1 registration might be converged, RMSE: {rmse_os1:.8f}")
                self.os1_transform_fixed = True
                self.final_transform_os1 = self.transform_os1
                self.transform_os1_counter = 0  # reset counter
            else:
                self.get_logger().info(f"OS1 registration in progress {self.transform_os1}, RMSE: {rmse_os1:.8f}")
        else:
            # Use the fixed transform if RMSE is below the threshold
            self.transform_os1 = self.final_transform_os1
            self.transform_os1_counter += 1
            self.get_logger().info(f"Using fixed OS1 transform:{self.transform_os1}, count: {self.transform_os1_counter}")

        if self.transform_os1 is None:
            self.get_logger().warn("Skipping merge and publish due to failed registration.")
            return

        #TODO: apply transformation to the pointcloud
        self.get_logger().info(f"OS1 transformation is converged for {self.transform_os1}")
        self.publish_transform(self.transform_os1, "os1", "velodyne")  # publish transform

        merged_pcd = #TODO: Merge
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
        self.merged_pub.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    calibrator = LidarCalibrator()
    rclpy.spin(calibrator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
