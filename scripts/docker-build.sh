#!/bin/bash
set -e
docker build --build-arg BASE_IMAGE="ubuntu:jammy" -f ./Dockerfile -t ros_builder:latest-jammy . 
docker build --build-arg BASE_IMAGE="nvidia/cuda:11.7.0-devel-ubuntu22.04" -f ./Dockerfile -t ros_builder:latest-cuda-jammy .
