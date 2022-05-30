# ROS Builder

This builds a base ROS 2 docker image used for building Greenroom packages.

## Features 
* Greenroom's `bloom` [fork](https://github.com/Greenroom-Robotics/bloom/tree/david_revay/sc-4323/version-pinning-in-package-xml) with `version_eq` support
* Greenroom's `rosdep` [fork](https://github.com/Greenroom-Robotics/rosdep/tree/david_revay/sc-4323/version-pinning-in-package-xml) with `version_eq` support
* `rosidl` with dynamic allocation backported from rolling (Galactic only)
* All base interfaces compiled with forked `rosidl` and `rosidl_generator_mypy`
