#!/bin/bash
set -e

# Squash and push DeepStream image
# Usage: ./squash-deepstream.sh <deepstream_version> <arch>

DEEPSTREAM_VERSION="${1:-8.0}"
ARCH="${2:-amd64}"

BASE_IMAGE="nvcr.io/nvidia/deepstream:${DEEPSTREAM_VERSION}-triton-multiarch"
OUTPUT_TAG="ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch-${ARCH}"
TEMP_IMAGE="deepstream-cleaned-temp:${ARCH}"

echo "Pulling DeepStream base image for ${ARCH}..."
docker pull --platform linux/${ARCH} ${BASE_IMAGE}

echo "Inspecting image layers..."
LAYER_COUNT=$(docker inspect ${BASE_IMAGE} | jq '.[0].RootFS.Layers | length')
echo "Image has ${LAYER_COUNT} layers"

# Clean up unused components from the image
echo "Creating temporary container to clean up unused components..."
CONTAINER_ID=$(docker create --platform linux/${ARCH} ${BASE_IMAGE} sleep infinity)

echo "Starting container..."
docker start ${CONTAINER_ID}

echo "Copying cleanup script into container..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker cp "${SCRIPT_DIR}/cleanup-deepstream-contents.sh" ${CONTAINER_ID}:/tmp/cleanup-deepstream-contents.sh

echo "Running cleanup script..."
docker exec ${CONTAINER_ID} bash /tmp/cleanup-deepstream-contents.sh

echo "Committing cleaned container to temporary image..."
docker commit ${CONTAINER_ID} ${TEMP_IMAGE}

echo "Stopping and removing temporary container..."
docker stop ${CONTAINER_ID}
docker rm ${CONTAINER_ID}

echo "Squashing cleaned image..."
docker-squash -t ${OUTPUT_TAG} ${TEMP_IMAGE}

echo "Removing temporary image..."
docker rmi ${TEMP_IMAGE}

echo "Pushing squashed image to registry: ${OUTPUT_TAG}"
docker push ${OUTPUT_TAG}

echo "Successfully cleaned, squashed and pushed ${OUTPUT_TAG}"
