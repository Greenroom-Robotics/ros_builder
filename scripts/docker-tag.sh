echo "Tagging version: $1"
echo "ROS version: $2"

echo "ghcr.io/greenroom-robotics/ros_builder:$2-$1"
docker tag ros_builder:latest-$2 ghcr.io/greenroom-robotics/ros_builder:$2-$1
echo "ghcr.io/greenroom-robotics/ros_builder:$2-latest"
docker tag ros_builder:latest-$2 ghcr.io/greenroom-robotics/ros_builder:$2-latest

echo "ghcr.io/greenroom-robotics/ros_builder:$2-$1-cuda"
docker tag ros_builder:latest-$2-cuda ghcr.io/greenroom-robotics/ros_builder:$2-$1-cuda
echo "ghcr.io/greenroom-robotics/ros_builder:$2-latest-cuda"
docker tag ros_builder:latest-$2-cuda ghcr.io/greenroom-robotics/ros_builder:$2-latest-cuda
