#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  
from std_msgs.msg import Float32  
from process import DeviationProcessor  
import threading
import time


class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor')

        # Declare parameters for topics
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('laser_topic', '')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('start_signal_topic', 'map_name')  

        # Get topic names from parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.laser_topic = self.get_parameter('laser_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.start_signal_topic = self.get_parameter('start_signal_topic').value  

        
        self.processor = DeviationProcessor()

        
        self.velocity_pub = self.create_publisher(Twist, self.cmd_vel_topic, 15)

        
        self.laser_deviation_subscriber = self.create_subscription(
            Float32, self.laser_topic, self.laser_deviation_callback, 1)
        self.image_deviation_subscriber = self.create_subscription(
            Float32, self.image_topic, self.image_deviation_callback, 1)

        
        self.start_signal_subscriber = self.create_subscription(
            String, self.start_signal_topic, self.start_signal_callback, 10)

       
        self.laser_deviation = 0.0
        self.image_deviation = 0.0
        self.angular_correction = 0.0
        self.received_name = None  

        
        self.laser_updated = False
        self.image_updated = False
        self.start_processing = False  

        self.get_logger().info("Processing Node succesfully initialised")

    def laser_deviation_callback(self, msg: Float32):
        self.laser_deviation = msg.data
        self.laser_updated = True
        self.get_logger().info(f"Laser deviation updated: {self.laser_deviation}")

    def image_deviation_callback(self, msg: Float32):
        self.image_deviation = msg.data
        self.image_updated = True
        self.get_logger().info(f"Image deviation updated: {self.image_deviation}")

    def start_signal_callback(self, msg: String):
        self.received_name = msg.data 
        self.start_processing = True
        self.get_logger().info(f"Received Map with name: {self.received_name}")

    def velocity_callback(self, msg: Twist):
        if not self.start_processing:
            return  
        
        if self.laser_updated or self.image_updated:
            combined_deviation = self.processor.combine_deviations(
                self.laser_deviation, self.image_deviation
            )
            self.angular_correction = self.processor.compute_velocity_correction(combined_deviation)
            self.laser_updated = False
            self.image_updated = False
        else:
            self.angular_correction = 0.0

        corrected_velocity = Twist()
        corrected_velocity.linear = msg.linear
        corrected_velocity.angular.x = msg.angular.x
        corrected_velocity.angular.y = msg.angular.y
        corrected_velocity.angular.z = msg.angular.z + self.angular_correction

        self.velocity_pub.publish(corrected_velocity)
        self.get_logger().info(f"Corrected Angular : {corrected_velocity.angular.z}")

    def read_rosbag_and_publish(self, bag_path):
        if not self.start_processing:
            return  

        try:
            bag_reader = self.processor.create_bag_reader(bag_path)
            self.processor.process_rosbag(
                bag_reader,
                self.cmd_vel_topic,
                self.velocity_callback
            )
            self.get_logger().info("Finished processing rosbag.")
            self.start_processing = False
        except Exception as e:
            self.get_logger().error(f"Error reading rosbag: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()

    def start_processing_bag():
        while True:
            if node.received_name:
                node.get_logger().info(f"Processing rosbag: {node.received_name}")
                node.read_rosbag_and_publish(node.received_name)
                #########
                node.received_name = None
            else:
                node.get_logger().info("Waiting for a Map to process...")
                time.sleep(5)  

    # Start rosbag processing in a background thread
    bag_thread = threading.Thread(target=start_processing_bag, daemon=True)
    bag_thread.start()

    
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down processor node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
