# Move to pose controller for ROS 2
#
# Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py
#
# Copyright 2023 Martin Nievas

import rclpy
from rclpy.node import Node
import numpy as np

from move_to_pose_ros.path_finder_controller import PathFinderController

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.global_path_publisher_= self.create_publisher(MarkerArray, 'global_path', 10)
        self.local_path_publisher_ = self.create_publisher(MarkerArray, 'local_path', 10)

        self.initial_pose = False
        self.prev_time = self.get_clock().now().to_msg()

        # simulation parameters
        self.controller = PathFinderController(9, 15, 3)
        self.dt = 0.01

        # Robot specifications
        self.MAX_LINEAR_SPEED = 1.5
        self.MAX_ANGULAR_SPEED = 0.5
        self.x_start = 0.0
        self.y_start = 0.0
        self.theta_start = 0.0
        self.x_goal = 2.0
        self.y_goal = 5.0
        self.theta_goal = 0.0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        theta = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

        if self.initial_pose == False:
            self.x_start = x
            self.y_start = y
            self.initial_pose = True

        x_diff = self.x_goal - x
        y_diff = self.y_goal - y

        x_traj, y_traj = [], []
        dt = msg.header.stamp.sec - self.prev_time.sec

        rho = np.hypot(x_diff, y_diff)
        self.get_logger().info('rho: "%s"' % rho)
        self.get_logger().info('x: "%s"' % x)
        self.get_logger().info('y: "%s"' % y)
        if rho > 1.0:
            x_diff = self.x_goal - x
            y_diff = self.y_goal - y

            rho, v, w = self.controller.calc_control_command(
                x_diff, y_diff, theta, self.theta_goal)

            if abs(v) > self.MAX_LINEAR_SPEED:
                v = np.sign(v) * self.MAX_LINEAR_SPEED

            if abs(w) > self.MAX_ANGULAR_SPEED:
                w = np.sign(w) * self.MAX_ANGULAR_SPEED

            cmd_vel = Twist()
            cmd_vel.linear.x = v
            cmd_vel.angular.z = w
            self.publisher_.publish(cmd_vel)
            self.get_logger().info('Publishing: "%s"' % cmd_vel)
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_.publish(cmd_vel)

    def generate_markerarray_message(self, path_x, path_y, type="global"):
        marker_array = MarkerArray()
        self.get_logger().info('marker size: "%s"' % len(path_x))
        for i in range(len(path_x[:-1])):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.pose.position.x = path_x[i]
            marker.pose.position.y = path_y[i]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            if type == "local":
                marker.scale.x = 0.03
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif type == "global":
                marker.scale.x = 0.01
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 1.0

            # marker line points
            marker.points = []

            # first point
            first_line_point = Point()
            first_line_point.x = path_x[i]
            first_line_point.y = path_y[i]
            first_line_point.z = 0.0
            marker.points.append(first_line_point)
            # second point
            second_line_point = Point()
            second_line_point.x = path_x[i+1]
            second_line_point.y = path_y[i+1]
            second_line_point.z = 0.0
            marker.points.append(second_line_point)

            marker_array.markers.append(marker)
        return marker_array


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()