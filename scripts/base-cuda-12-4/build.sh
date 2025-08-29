#!/bin/bash
# Builds a base image with Ubuntu 24.04 with CUDA 12.4 to be used by ros_builder.
# This configuration is not available on nvcr.io and there are no compatible .debs.
# This is required to support Ocean Infinity's VM.

# wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/linux-x86_64/cudnn-linux-x86_64-9.1.0.70_cuda12-archive.tar.xz
# wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.5.0/tars/TensorRT-10.5.0.18.Linux.x86_64-gnu.cuda-12.6.tar.gz

docker buildx build \
    -f Dockerfile.cuda12-4 \
    --platform linux/amd64 \
    --progress=plain \
    -t "ghcr.io/greenroom-robotics/cuda:12-4-amd64" \
    .

docker push "ghcr.io/greenroom-robotics/cuda:12-4-amd64"

docker manifest create ghcr.io/greenroom-robotics/cuda:12-4 \
    ghcr.io/greenroom-robotics/cuda:12-4-amd64

docker manifest push ghcr.io/greenroom-robotics/cuda:12-4

echo "Build and manifest creation completed successfully!"