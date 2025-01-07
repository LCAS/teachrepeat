#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
import cv2
import glob
import time
from deviation import calculate_laser_deviation, calculate_image_deviation
import numpy as np
import json
import csv
from teachrepeat.action import MapRepeat 

class DeviationCalculator(Node):
    def __init__(self):
        super().__init__('deviation_calculator')

        # Declare parameters
        self.declare_parameter('scan_folder', '')
        self.declare_parameter('image_folder', '')
        self.declare_parameter('deviation_interval', 0.0)
        self.declare_parameter('laser_scan_topic', '')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('depth_image_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('laser_deviation_topic', '')
        self.declare_parameter('image_deviation_topic', '')

        # Retrieve initial parameters
        self.scan_folder = self.get_parameter('scan_folder').value
        self.image_folder = self.get_parameter('image_folder').value
        self.deviation_interval = self.get_parameter('deviation_interval').value
        self.laser_scan_topic = self.get_parameter('laser_scan_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.laser_deviation_topic = self.get_parameter('laser_deviation_topic').value
        self.image_deviation_topic = self.get_parameter('image_deviation_topic').value

        # Create CvBridge object
        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(LaserScan, self.laser_scan_topic, self.scan_callback, 1)
        self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 1)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Publishers
        self.laser_deviation_publisher = self.create_publisher(Float32, self.laser_deviation_topic, 1)
        self.image_deviation_publisher = self.create_publisher(Float32, self.image_deviation_topic, 1)

        # Timer
        self.timer = self.create_timer(self.deviation_interval, self.calculate_and_publish_deviations)

        # Initialize variables
        self.last_ros_scan = None
        self.last_ros_image = None
        self.depth_image = None
        self.last_position = (0.0, 0.0)
        self.scan_files = sorted(glob.glob(os.path.join(self.scan_folder, 'lidar_scan_*.json')))
        self.image_files = sorted(glob.glob(os.path.join(self.image_folder, 'image_*.png')))
        self.scan_index = 0
        self.image_index = 0
        self.odom_record = False
        self.check_file = True

        


        # Action server
        self.action_server = ActionServer(
            self,
            MapRepeat,
            'repeat_map',
            self.handle_goal
        )

        self.get_logger().info("Deviation Node succesfully initialised")
        self.get_logger().info("Ready!!!")

    def handle_goal(self, goal_handle):
        self.get_logger().info("Received goal to configure repeat parameters")

        # Update parameters from goal
        goal = goal_handle.request
        self.scan_folder = goal.output_folder
        self.image_folder = goal.output_folder
        self.deviation_interval = goal.record_interval
        
        output_folder_msg = String()
        output_folder_msg.data = self.scan_folder  
        self.create_publisher(String, 'map_name', 1).publish(output_folder_msg)
        self.get_logger().info(f"Published output folder: {self.scan_folder}")
        self.odom_record = True

        # Restart timer with new interval
        self.timer.cancel()
        self.timer = self.create_timer(self.deviation_interval, self.calculate_and_publish_deviations)

        # Update file lists
        self.scan_files = sorted(glob.glob(os.path.join(self.scan_folder, 'lidar_scan_*.json')))
        self.image_files = sorted(glob.glob(os.path.join(self.image_folder, 'image_*.png')))
        self.scan_index = 0
        self.image_index = 0

        goal_handle.succeed()
        return MapRepeat.Result(success=True)

    def scan_callback(self, msg):
        try:
            ranges = np.array(msg.ranges)
            ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
            self.last_ros_scan = ranges
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

    def image_callback(self, msg):
        try:
            ros_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_ros_image = ros_image
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image: {e}")

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
        self.depth_image = depth_image

    def odom_callback(self, msg):
        try:
            position = msg.pose.pose.position
            x, y = position.x, position.y
            self.last_position = (x, y)
            if self.odom_record:
                self.odom_output_file = os.path.join(self.scan_folder, "odom_data2.csv")
                if self.check_file:
                    if os.path.exists(self.odom_output_file):
                        os.remove(self.odom_output_file)
                    self.check_file = False
                    
                with open(self.odom_output_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    # Write the data
                    writer.writerow([time.time(), x, y])

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def calculate_and_publish_deviations(self):
        if self.last_ros_scan is not None and self.last_ros_image is not None and self.depth_image is not None:
            try:
                if self.scan_index < len(self.scan_files) and self.image_index < len(self.image_files):
                    saved_scan_path = self.scan_files[self.scan_index]
                    self.scan_index += 1
                    with open(saved_scan_path, 'r') as f:
                        saved_scan = json.load(f)
                        saved_ranges = np.array(saved_scan.get('ranges', []))
                        saved_ranges = saved_ranges[~np.isnan(saved_ranges) & ~np.isinf(saved_ranges)]

                    laser_deviation = calculate_laser_deviation(saved_ranges, self.last_ros_scan, self.last_position)
                    self.laser_deviation_publisher.publish(Float32(data=laser_deviation))
                    #self.get_logger().info(f"Published laser deviation: {laser_deviation:.2f} meters")

                    saved_image_path = self.image_files[self.image_index]
                    self.image_index += 1
                    saved_image = cv2.imread(saved_image_path)

                    image_deviation = calculate_image_deviation(saved_image, self.last_ros_image, self.depth_image, self.last_position)
                    self.image_deviation_publisher.publish(Float32(data=image_deviation))
                    #self.get_logger().info(f"Published image deviation: {image_deviation:.2f} meters")
            except Exception as e:
                self.get_logger().error(f"Error during deviation calculation: {e}")

def main(args=None):
    rclpy.init(args=args)
    deviation_calculator = DeviationCalculator()
    rclpy.spin(deviation_calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
