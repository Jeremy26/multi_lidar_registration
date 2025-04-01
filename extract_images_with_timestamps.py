#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Extract images and frame-wise timestamps from a ROS bag.

This script extracts images from the specified image topic in a ROS bag
and saves them as PNG files in the output directory. In addition, it generates
a CSV file ("timestamps.csv") in the same directory that logs the frame index
and the timestamp (in seconds) of each extracted image. This CSV can be used
later to synchronize the images with lidar data.
"""

import os
import argparse
import csv
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    parser = argparse.ArgumentParser(description="Extract images and timestamps from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag file.")
    parser.add_argument("output_dir", help="Directory to save extracted images and timestamps.")
    parser.add_argument("image_topic", help="ROS topic for the images (e.g. /cam_1/color/image_raw).")
    args = parser.parse_args()

    print("Extracting images from {} on topic {} into {}".format(args.bag_file, args.image_topic, args.output_dir))

    # Create the output directory if it doesn't exist.
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # Open a CSV file to log frame indices and timestamps.
    csv_filename = os.path.join(args.output_dir, "timestamps.csv")
    csv_file = open(csv_filename, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["frame", "timestamp_sec"])

    # Open the ROS bag.
    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0

    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        try:
            # Convert the ROS Image message to an OpenCV image.
            # For color images, use "bgr8"; for depth, you might use "passthrough".
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            print("Error converting image on frame {}: {}".format(count, e))
            continue

        # Save the image as a PNG file with a filename that includes the frame index.
        image_filename = os.path.join(args.output_dir, "frame{:06d}.png".format(count))
        if cv2.imwrite(image_filename, cv_img):
            print("Saved image {}".format(image_filename))
        else:
            print("Failed to save image {}".format(image_filename))
            continue

        # Write the frame index and timestamp (converted to seconds) to the CSV.
        csv_writer.writerow([count, t.to_sec()])
        count += 1

    bag.close()
    csv_file.close()
    print("Extraction complete. Total frames extracted: {}".format(count))

if __name__ == '__main__':
    main()
