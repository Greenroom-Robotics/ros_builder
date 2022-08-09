echo "Tagging version: $1"
echo "ghcr.io/greenroom-robotics/ros_builder:$1"
docker tag ros_builder:latest ghcr.io/greenroom-robotics/ros_builder:$1
echo "ghcr.io/greenroom-robotics/ros_builder:latest"
docker tag ros_builder:latest ghcr.io/greenroom-robotics/ros_builder:latest

echo "ghcr.io/greenroom-robotics/ros_builder:cuda-$1"
docker tag ros_builder:cuda-latest ghcr.io/greenroom-robotics/ros_builder:cuda-$1
echo "ghcr.io/greenroom-robotics/ros_builder:cuda-latest"
docker tag ros_builder:cuda-latest ghcr.io/greenroom-robotics/ros_builder:cuda-latest