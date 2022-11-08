echo "Tagging version: $1"
echo "ROS version: $2"

echo "ghcr.io/greenroom-robotics/ros_builder:$1"
docker tag ros_builder:latest-$2 ghcr.io/greenroom-robotics/ros_builder:$1-$2
echo "ghcr.io/greenroom-robotics/ros_builder:latest"
docker tag ros_builder:latest-$2 ghcr.io/greenroom-robotics/ros_builder:latest-$2

echo "ghcr.io/greenroom-robotics/ros_builder:cuda-$1"
docker tag ros_builder:latest-$2-cuda ghcr.io/greenroom-robotics/ros_builder:$1-$2-cuda
echo "ghcr.io/greenroom-robotics/ros_builder:cuda-latest"
docker tag ros_builder:latest-$2-cuda ghcr.io/greenroom-robotics/ros_builder:latest-$2-cuda
