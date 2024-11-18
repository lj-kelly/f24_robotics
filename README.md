# Repository for UA CS460/560 - Fall 2024

**Links to ROS resources**

([Short video overview](
https://vimeo.com/639236696))


Tutorials to complete: 

1. ([CLI Tools Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html))
2. ([Beginner Client Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html))


***Homework Assignment 1***

[Webots/ROS2/Heuristic Search](./Homework1/Assignment.md)



***Homework Assignment 4***

Algorithm sends robot in a straight line to a wall in search of an apriltag. Once at the wall, rotates almost one full rotation and proceeds in a straight line until a new wall is found.

RUN WEBOTS
*open first terminal
cd f24_robotics
colcon build
source install/setup.bash
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py

*open second terminal
cd f24_robotics
ros2 run webots_ros2_homework1_python webots_ros2_homework1_python

these ros2 packages are needed as well.
    >ros2 run v4l2_camera v4l2_camera_node
    >ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file  `ros2 pkg prefix apriltag_ros`/cfg/>tags_36h11.yaml
    >ros2 run rqt_image_view rqt_image_view
    >ros2 topic echo /detections