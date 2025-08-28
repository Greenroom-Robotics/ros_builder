#!/bin/bash

docker buildx build \
    -f Dockerfile.cuda12-4 \
    --platform linux/amd64 \
    --progress=plain \
    -t "ghcr.io/greenroom-robotics/cuda:12-4" \
    .

echo "Build completed successfully!"