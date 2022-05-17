# ROS Builder

This builds a basic ROS2 docker image used for Greenroom Projects

## Features 
* Greenroom's `bloom` [fork](https://github.com/Greenroom-Robotics/bloom/commits/feature/fix-namespace)
* Greenroom's `rosdep` [fork](https://github.com/Greenroom-Robotics/rosdep)
* Greenroom's rosdep list
* `rosidl` with dynamic allocation backport
* `rosidl_defaults` with `rosidl_generator_mypy` support
* All base interfaces compiled with forked `rosidl` and `rosidl_generator_mypy`
