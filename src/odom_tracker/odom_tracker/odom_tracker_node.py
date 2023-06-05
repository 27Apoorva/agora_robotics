#!/usr/bin/python3
# Description: Main node for odom_tracker package. Comapres the odometry with the ground truth odometry and publishes the error.
# ROS2 Python package.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from sensor_msgs.msg import Imu
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import io
import cv2
from std_srvs.srv import SetBool
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped, Vector3
import tf2_geometry_msgs
# import tf.transformations as tft
import os

class OdomTracker(Node):
    """
    Main node for odom_analyzer package. Comapres the odometry with the ground truth odometry and publishes the error.
    """
    def __init__(self):
        super().__init__('odom_tracker_node')
        
        # Create tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.translational_error = []
        self.translational_error_wheel = []
        # Create a subscription to the wheel odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10)

        # Create a publisher for the ground truth odometry
        self.ground_truth_odom_publisher = self.create_publisher(
            Odometry,
            '/ground_truth_odom',
            10)

        # Create a publisher for the error
        self.error_publisher = self.create_publisher(
            Odometry,
            '/odom_error',
            10)
        self.t_publisher = self.create_publisher(Image, 'translation_error', 10)
        # Create a subscriber for the ground truth odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.ground_truth_odom_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Odometry,
            '/imu',
            self.imu_callback,
            10)
        # self.imu_publisher = self.create_publisher(
        #     Imu,
        #     '/imu_o',
        #     10
        # )
        
        self.ground_truth_odometry = Odometry()
        self.wheel_odom = Odometry()
        self.map_T_odom = Transform()
        self.cv_bridge = CvBridge()
        # service = rospy.Service('plot_and_save', SetBool, handle_plot_and_save)
        self.srv = self.create_service(SetBool, 'plot_and_save', self.handle_plot_and_save)


    
    def imu_callback(self, msg):
        # Swap the value of orientation
        temp = msg.orientation.w
        msg.orientation.w = msg.orientation.z
        msg.orientation.z = temp

        # Publish the modified IMU message
        self.imu_publisher.publish(msg)


    def odom_callback(self, msg):
        """
        Callback function for the wheel odometry subscription. Compares the wheel odometry with the ground truth odometry and publishes the error.
        """
        # Get the transform between the wheel odometry and the ground truth odometry
        try:
            transform = self.tf_buffer.lookup_transform(
                'libsurvive_world',
                'LHR-97093BF8',
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info('Failed to get transform between wheel_odom and odom.')
            return

        # Get the ground truth odometry
        ground_truth_odom = Odometry()
        ground_truth_odom.header.stamp = msg.header.stamp
        ground_truth_odom.header.frame_id = 'libsurvive_world'
        ground_truth_odom.child_frame_id = 'LHR-97093BF8'
        ground_truth_odom.pose.pose.position.x = transform.transform.translation.x
        ground_truth_odom.pose.pose.position.y = transform.transform.translation.y
        ground_truth_odom.pose.pose.position.z = transform.transform.translation.z
        ground_truth_odom.pose.pose.orientation.x = transform.transform.rotation.x
        ground_truth_odom.pose.pose.orientation.y = transform.transform.rotation.y
        ground_truth_odom.pose.pose.orientation.z = transform.transform.rotation.z
        ground_truth_odom.pose.pose.orientation.w = transform.transform.rotation.w

        # Publish the ground truth odometry
        self.ground_truth_odom_publisher.publish(ground_truth_odom)
        
        # Save the ground truth odometry
        self.wheel_odom = msg
        self.ground_truth_odometry = ground_truth_odom

    def handle_plot_and_save(self, request, response):
        # values = request.values  # Assuming the request has a field named 'values' which is a list

        # Plot the values
        plt.plot(self.translational_error, label ='filtered_odom_error')
        plt.plot(self.translational_error_wheel, label ='wheel_odom_error')
        plt.xlabel('Time')
        plt.ylabel('Error')
        plt.legend()
        plt.title('Plot of Values')

        # Save the plot as an image file
        image_path = 'output.png'
        plt.savefig(image_path)

        # Close the plot
        plt.close()

        # response = SetBoolResponse()
        response.success = True
        # response.message = 'Plot and image saved successfully'
        return response
    
    def convert_quat_to_euler(self, quaternion):
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(2 * (qw * qy - qz * qx))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw)
        # self.get_logger().info('yaw: %f' % yaw)
        return yaw


    def ground_truth_odom_callback(self, msg):
        try:
            self.map_T_odom = self.tf_buffer.lookup_transform(
                'libsurvive_world',
                'odom',
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info('Failed to get transform between libsurvive_world and odom.')
            return
        
        map_T_odom_yaw = self.convert_quat_to_euler(self.map_T_odom.transform.rotation)
        map_T_odom_rot = np.array([[np.cos(map_T_odom_yaw), -np.sin(map_T_odom_yaw)],
                            [np.sin(map_T_odom_yaw), np.cos(map_T_odom_yaw)]])
        map_T_odom_translation = np.array([
            self.map_T_odom.transform.translation.x,
            self.map_T_odom.transform.translation.y,
            1
        ])
        map_T_odom_arr = np.identity(3)
        
        map_T_odom_arr[:2, :2] = map_T_odom_rot
        map_T_odom_arr[:3, 2] = map_T_odom_translation

        wheel_odom_yaw = self.convert_quat_to_euler(self.wheel_odom.pose.pose.orientation)
        wheel_odom_rot = np.array([[np.cos(wheel_odom_yaw), -np.sin(wheel_odom_yaw)],
                            [np.sin(wheel_odom_yaw), np.cos(wheel_odom_yaw)]])
        wheel_odom_translation = np.array([
            self.wheel_odom.pose.pose.position.x,
            self.wheel_odom.pose.pose.position.y,
            1
        ])
        wheel_odom_array = np.identity(3)
        wheel_odom_array[:2, :2] = wheel_odom_rot
        wheel_odom_array[:3, 2] = wheel_odom_translation
        
        # Perform the dot product of the two transformation matrices
        wheel_odom_result = np.dot(map_T_odom_arr, wheel_odom_array)
        # import pdb; pdb.set_trace()
        # self.get_logger().info("map_T_odom")
        # self.get_logger().info('x: %f' % map_T_odom_translation[0])
        # self.get_logger().info('y: %f' % map_T_odom_translation[1])
        # self.get_logger().info("filtered odom")
        # self.get_logger().info('x: %f' % filtered_odom_translation[0])
        # self.get_logger().info('y: %f' % filtered_odom_translation[1])
        
        filtered_odom_yaw = self.convert_quat_to_euler(msg.pose.pose.orientation)
        filtered_odom_rot = np.array([[np.cos(filtered_odom_yaw), -np.sin(filtered_odom_yaw)],
                            [np.sin(filtered_odom_yaw), np.cos(filtered_odom_yaw)]])
        filtered_odom_translation = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            1
        ])
        filtered_odom_array = np.identity(3)
        filtered_odom_array[:2, :2] = filtered_odom_rot
        filtered_odom_array[:3, 2] = filtered_odom_translation

        filtered_odom_result = np.dot(map_T_odom_arr, filtered_odom_array)

        # self.get_logger().info("ground truth")
        # self.get_logger().info('x: %f' % self.ground_truth_odometry.pose.pose.position.x)
        # self.get_logger().info('y: %f' % self.ground_truth_odometry.pose.pose.position.y)

        # self.get_logger().info("odom filtered truth")
        # # import pdb; pdb.set_trace()
        # self.get_logger().info('x: %f' % filtered_odom_result[0,2])
        # self.get_logger().info('y: %f' % filtered_odom_result[1,2])

        # self.get_logger().info("wheel odom truth")
        # # import pdb; pdb.set_trace()
        # self.get_logger().info('x: %f' % wheel_odom_result[0,2])
        # self.get_logger().info('y: %f' % wheel_odom_result[1,2])

        # distance = np.sqrt(error.pose.pose.position.x**2 + error.pose.pose.position.y**2)
        # self.get_logger().info('gt distance: %f' % distance)

        filtered_error_x = self.ground_truth_odometry.pose.pose.position.x - filtered_odom_result[0,2]
        filtered_error_y = self.ground_truth_odometry.pose.pose.position.y - filtered_odom_result[1,2]
        # self.get_logger().info('wheel_odom x: %f' % error_x)
        # self.get_logger().info('wheel_odom y: %f' % error_y)
        filtered_distance = np.sqrt(filtered_error_x**2 + filtered_error_y**2)
        self.get_logger().info('filtered distance: %f' % filtered_distance)
        odom_error_x = self.ground_truth_odometry.pose.pose.position.x - wheel_odom_result[0,2]
        odom_error_y = self.ground_truth_odometry.pose.pose.position.y - wheel_odom_result[1,2]
        # self.get_logger().info('wheel_odom x: %f' % error_x)
        # self.get_logger().info('wheel_odom y: %f' % error_y)
        odom_distance = np.sqrt(odom_error_x**2 + odom_error_y**2)
        self.get_logger().info('odom distance: %f \n' % odom_distance)
        delta = odom_distance - filtered_distance
        self.get_logger().info('delta: %f \n' % delta)
        # self.error_publisher.publish(error)
        if filtered_distance > 1000:
            print("distance greater")
            return
        else:
            self.translational_error.append(filtered_distance)
            self.translational_error_wheel.append(odom_distance)
        # plt.plot(self.translational_error)
        # plt.xlabel('Time')
        # plt.ylabel('Translational Error')
        # plt.title('Translational Error between Poses')
        # fig, ax = plt.subplots()
        # # ax.plot(self.translational_error)
        # ax.set_xlabel('Time')
        # ax.set_ylabel('Error')
        # ax.set_title('Translational Error between Poses')
        # image_path = 'output.png'
        # fig.savefig(image_path)
        # plt.show()
         # Convert the plot to an image and publish it
        # Convert the Matplotlib figure to an image format (e.g., PNG)
        # image_format = 'png'
        # image_data = self.fig_to_image(fig, image_format)

        # # Create an Image message and publish it
        # image_msg = self.cv_bridge.cv2_to_imgmsg(image_data, encoding='passthrough')
        # self.t_publisher.publish(image_msg)
    def fig_to_image(self, fig, image_format='png'):
        # Convert the Matplotlib figure to an image buffer
        buf = io.BytesIO()
        fig.savefig(buf, format=image_format)
        buf.seek(0)

        # Read the image buffer using OpenCV
        image_data = cv2.imdecode(np.frombuffer(buf.getvalue(), dtype=np.uint8), -1)

        return image_data



def main(args=None):
    rclpy.init(args=args)

    odom_tracker = OdomTracker()

    rclpy.spin(odom_tracker)

    odom_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
