# ROS Builder

This builds a base ROS 2 docker image used for building Greenroom packages.

## Features 
* Greenroom's `bloom` [fork](https://github.com/Greenroom-Robotics/bloom/)
* Greenroom's `rosdep` [fork](https://github.com/Greenroom-Robotics/rosdep/)
* `rosidl` with custom memory allocator support backported from humble (required for Galactic only)
* All base interfaces generated and compiled with forked `rosidl` and `rosidl_generator_mypy`

## Versions

There are 4 different versions of `ros_builder`. See [ghcr](https://github.com/Greenroom-Robotics/ros_builder/pkgs/container/ros_builder). These are

* ` ghcr.io/greenroom-robotics/ros_builder:humble-lastest` **AMD64**
* ` ghcr.io/greenroom-robotics/ros_builder:humble-lastest-cuda` **AMD64**
* ` ghcr.io/greenroom-robotics/ros_builder:humble-lastest` **ARM64** - *Note this does not include valcanexus*
* ` ghcr.io/greenroom-robotics/ros_builder:humble-lastest-cuda` **ARM64** - *Note this does not include valcanexus*

## Releasing

### Github Action

*This will be slow to build but fast to upload.*

Trigger the [publish.yml](./.github/workflows/publish.yml) github action

### Local

*This will be fast(er) to build but slow to upload.*

* `yarn version --patch|minor|major`
* `yarn docker:build-and-push`
* `git push`