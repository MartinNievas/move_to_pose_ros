# Move to pose controller for ROS 2
#
# Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py
#
# Copyright 2023 Martin Nievas

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse,GoalResponse
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from move_to_pose_ros.path_finder_controller import PathFinderController
from move_to_pose_ros.visualization_helper import generate_markerarray_message
from action_move_to_pose_interface.action import MoveToPose

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller Initiated')
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.action_server_ = ActionServer(
            self,
            MoveToPose,
            'MoveToPose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)

        self.initial_pose = False
        self.prev_time = self.get_clock().now().to_msg()
        self.curren_pose_x = 0.0
        self.curren_pose_y = 0.0
        self.distance_to_goal = 0.0
        self.on_target = False

        # simulation parameters
        self.controller = PathFinderController(9, 15, 3)
        self.dt = 0.01

        # Robot specifications
        self.MAX_LINEAR_SPEED = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.get_logger().info('Robot velocity linear: %f, angular: %f' % (self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED))
        self.x_start = 0.0
        self.y_start = 0.0
        self.theta_start = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

    def odom_callback(self, msg):
        self.MAX_LINEAR_SPEED = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        theta = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

        self.curren_pose_x = x
        self.curren_pose_y = y

        if self.initial_pose == False:
            self.x_start = x
            self.y_start = y
            self.initial_pose = True

        x_diff = self.x_goal - x
        y_diff = self.y_goal - y

        x_traj, y_traj = [], []
        dt = msg.header.stamp.sec - self.prev_time.sec

        rho = np.hypot(x_diff, y_diff)
        self.distance_to_goal = rho

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
            self.on_target = False
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_.publish(cmd_vel)
            self.on_target = True

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Deferring execution...')
        self._goal_handle = goal_handle
        # Inicio un nuevo goal
        self.x_goal = goal_handle.request.goal_pose.position.x
        self.y_goal = goal_handle.request.goal_pose.position.y
        self.on_target = False
        self.initial_pose = False
        self.get_logger().info('Moving to goal...')
        self._timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        # Execute the defered goal
        if self._goal_handle is not None:
            if self.on_target == False:
                self.provide_feedback(self._goal_handle)
            else:
                self._timer.cancel()
                self._goal_handle.execute()

    def provide_feedback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveToPose.Feedback()
        feedback_msg.distance_to_goal = self.distance_to_goal
        goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Goal reached!')
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.final_distance = self.distance_to_goal
        return result


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(controller_node, executor=executor)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()