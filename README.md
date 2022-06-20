# ROS Builder

This builds a base ROS 2 docker image used for building Greenroom packages.

## Features 
* Greenroom's `bloom` [fork](https://github.com/Greenroom-Robotics/bloom/)
* Greenroom's `rosdep` [fork](https://github.com/Greenroom-Robotics/rosdep/)
* `rosidl` with custom memory allocator support backported from humble (required for Galactic only)
* All base interfaces generated and compiled with forked `rosidl` and `rosidl_generator_mypy`
