#!/bin/bash
catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release -DSAVE_TIMES=ON
catkin build
