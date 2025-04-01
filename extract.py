#!/usr/bin/env python3
import os
import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import csv

# Define the topics for the three standard lidars:
topics = {
    "velodyne": "/velodyne_points",         # VLP-16 Lidar
    "os0":       "/os_cloud_node/points",     # OS0 Lidar
    "os1":       "/os_cloud_nodee/points"     # OS1 Lidar
}

# Create separate output directories for each sensor
output_dirs = {}
for key in topics:
    out_dir = f"pcd_new_{key}"
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    output_dirs[key] = out_dir

# Create CSV files to log timestamps for each sensor for synchronization later.
csv_files = {}
for key in topics:
    csv_path = os.path.join(output_dirs[key], f"{key}_timestamps.csv")
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["frame_idx", "timestamp_sec"])
    csv_files[key] = (csv_file, writer)

# Path to your ROS bag file
bag_path = "sensor_data_new_2.bag"  # adjust if needed
print("Opening bag file:", bag_path)
bag = rosbag.Bag(bag_path, "r")

# Initialize a dictionary to count frames per sensor
frame_counts = {key: 0 for key in topics}

# Iterate over messages for the specified lidar topics
for topic, msg, t in bag.read_messages(topics=list(topics.values())):
    # Determine which sensor this message belongs to
    sensor_key = None
    for key, topic_name in topics.items():
        if topic == topic_name:
            sensor_key = key
            break
    if sensor_key is None:
        continue

    # Convert the PointCloud2 message to a list of (x, y, z) points
    points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
    if not points_list:
        print(f"Frame {frame_counts[sensor_key]} on {sensor_key}: no points, skipping.")
        continue
    points = np.array(points_list)

    # Create an Open3D point cloud and assign the points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Build a filename that includes the sensor name and frame index
    frame_idx = frame_counts[sensor_key]
    filename = os.path.join(output_dirs[sensor_key], f"{sensor_key}_frame_{frame_idx:04d}.pcd")
    
    # Save the point cloud to a PCD file
    if o3d.io.write_point_cloud(filename, pcd):
        print(f"Saved {filename} with {len(points)} points.")
    else:
        print(f"Failed to save {filename}")

    # Log the timestamp (convert ROS time to seconds)
    csv_files[sensor_key][1].writerow([frame_idx, t.to_sec()])

    frame_counts[sensor_key] += 1

bag.close()

# Close CSV files
for key, (csv_file, _) in csv_files.items():
    csv_file.close()

print("Extraction complete. Frame counts per sensor:", frame_counts)
