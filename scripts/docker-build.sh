docker build --build-arg BASE_IMAGE="ubuntu:focal" -f ./Dockerfile -t ros_builder:latest . 
docker build --build-arg BASE_IMAGE="nvidia/cuda:11.3.1-devel-ubuntu20.04" -f ./Dockerfile -t ros_builder:latest-cuda .
