#!/bin/bash
set -e

# Squash and push DeepStream image
# Usage: ./squash-deepstream.sh <deepstream_version> <arch>

DEEPSTREAM_VERSION="${1:-8.0}"
ARCH="${2:-amd64}"

BASE_IMAGE="nvcr.io/nvidia/deepstream:${DEEPSTREAM_VERSION}-triton-multiarch"
OUTPUT_TAG="ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch-${ARCH}"

echo "Pulling DeepStream base image for ${ARCH}..."
docker pull --platform linux/${ARCH} ${BASE_IMAGE}

echo "Inspecting image layers..."
LAYER_COUNT=$(docker inspect ${BASE_IMAGE} | jq '.[0].RootFS.Layers | length')
echo "Image has ${LAYER_COUNT} layers"

docker-squash -t ${OUTPUT_TAG} ${BASE_IMAGE}

echo "Pushing squashed image to registry: ${OUTPUT_TAG}"
docker push ${OUTPUT_TAG}

echo "Successfully squashed and pushed ${OUTPUT_TAG}"
