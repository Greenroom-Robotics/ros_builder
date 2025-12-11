#!/bin/bash
set -e

# Create and push multi-arch manifest
# Usage: ./create-manifest.sh <deepstream_version>

DEEPSTREAM_VERSION="${1:-8.0}"

docker manifest create ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch \
  ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch-amd64 \
  ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch-arm64

docker manifest push ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch

echo "Successfully created and pushed multi-arch manifest"
