#!/bin/bash
set -e
export DOCKER_BUILDKIT=1
UBUNTU_VERSION="22.04"
UBUNTU_CODENAME="jammy"
CUDA_VERSION="cuda:11.7.0-devel-ubuntu22.04"

docker build --build-arg BASE_IMAGE="ubuntu:${UBUNTU_CODENAME}" -f ./Dockerfile -t ros_builder:latest . 
docker build --build-arg BASE_IMAGE="nvidia/cuda:${CUDA_VERSION}" -f ./Dockerfile -t ros_builder:latest-cuda .
