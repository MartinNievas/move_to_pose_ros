# move_to_pose_ros
A ROS 2 implementation of  PythonRobotics/Control/move_to_pose example

# Command-line Tools
move_to_pose_ros is a ROS 2 package that controls a robot base to move to a target pose.
The goal pose is given by an action client.

## Example
```bash
cd launch
ros2 launch controller.launch.py
```
In another terminal,
```bash
ros2 action send_goal /MoveToPose -f action_move_to_pose_interface/action/MoveToPose "goal_pose:
  position:
    x: 2.0
    y: 5.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## Published Topics
- `/cmd_vel` ([geometry_msgs/Twist](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg))
  - Velocity command to be sent to the robot base.

## Subscribed Topics
- `/odom` ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg))
  - Odometry information of the robot base.

## Parameters
- `max_linear_speed` (float, default: 0.5)
  - Maximum linear speed of the robot base.
- `max_angular_speed` (float, default: 0.5)
  - Maximum angular speed of the robot base.