echo "Tagging version: $1"
echo "ghcr.io/greenroom-robotics/ros_builder:galactic-$1"
docker tag ros_builder:latest ghcr.io/greenroom-robotics/ros_builder:galactic-$1
echo "ghcr.io/greenroom-robotics/ros_builder:galactic-latest"
docker tag ros_builder:latest ghcr.io/greenroom-robotics/ros_builder:galactic-latest

echo "ghcr.io/greenroom-robotics/ros_builder:galactic-cuda-$1"
docker tag ros_builder:latest-cuda ghcr.io/greenroom-robotics/ros_builder:galactic-$1-cuda
echo "ghcr.io/greenroom-robotics/ros_builder:galactic-cuda-latest"
docker tag ros_builder:latest-cuda ghcr.io/greenroom-robotics/ros_builder:galactic-latest-cuda