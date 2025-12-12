#!/bin/bash
set -e

# Squash and push DeepStream image
# Usage: ./squash-deepstream.sh <deepstream_version> <arch> [squash_flag]
# squash_flag: 'squash' (or any value) to perform the squash, or omit/any other value to skip.

DEEPSTREAM_VERSION="${1:-8.0}"
ARCH="${2:-amd64}"
SQUASH_FLAG="${3:-}" # New optional argument

BASE_IMAGE="nvcr.io/nvidia/deepstream:${DEEPSTREAM_VERSION}-triton-multiarch"
# Use the desired final tag name, which implies squashed/cleaned state
FINAL_OUTPUT_TAG="ghcr.io/greenroom-robotics/deepstream-squashed:${DEEPSTREAM_VERSION}-triton-multiarch-${ARCH}"
TEMP_IMAGE="deepstream-cleaned-temp:${ARCH}"

# Determine if squashing should happen
if [[ "${SQUASH_FLAG}" == "squash" ]]; then
    SQUASHING=true
    echo "Squash flag set. Image will be squashed and tagged as ${FINAL_OUTPUT_TAG}."
else
    SQUASHING=false
    echo "Squash flag NOT set. Image will be cleaned (but NOT squashed) and tagged as ${FINAL_OUTPUT_TAG}."
fi

echo "---"
echo "Pulling DeepStream base image for ${ARCH}..."
docker pull --platform linux/${ARCH} ${BASE_IMAGE}

echo "Inspecting image layers..."
LAYER_COUNT=$(docker inspect ${BASE_IMAGE} | jq '.[0].RootFS.Layers | length')
echo "Base image has ${LAYER_COUNT} layers"

# Build cleaned image using Dockerfile (This step always runs to remove temporary build artifacts)
echo "Building cleaned image using Dockerfile..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker build \
    --platform linux/${ARCH} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    -f "${SCRIPT_DIR}/Dockerfile.deepstream-cleanup" \
    -t ${TEMP_IMAGE} \
    "${SCRIPT_DIR}"

echo "---"

if $SQUASHING; then
    ## --- Squashing Path ---
    echo "Inspecting temporary image layers..."
    TEMP_LAYER_COUNT=$(docker inspect ${TEMP_IMAGE} | jq '.[0].RootFS.Layers | length')
    echo "Temporary image has ${TEMP_LAYER_COUNT} layers (original: ${LAYER_COUNT})"

    echo "Squashing image using docker-squash..."
    docker-squash -t ${FINAL_OUTPUT_TAG} ${TEMP_IMAGE}

else
    ## --- No-Squash Path ---
    echo "Skipping squash step. Retagging temporary image to final output tag..."
    # Simply tag the cleaned image built in the previous step
    docker tag ${TEMP_IMAGE} ${FINAL_OUTPUT_TAG}

fi

echo "Cleaning up temporary image: ${TEMP_IMAGE}"
docker rmi ${TEMP_IMAGE}

echo "---"
echo "Inspecting final image layers for ${FINAL_OUTPUT_TAG}..."
FINAL_LAYER_COUNT=$(docker inspect ${FINAL_OUTPUT_TAG} | jq '.[0].RootFS.Layers | length')
echo "Final image has ${FINAL_LAYER_COUNT} layers"

echo "Pushing final image to registry: ${FINAL_OUTPUT_TAG}"
docker push ${FINAL_OUTPUT_TAG}

echo "Successfully processed and pushed ${FINAL_OUTPUT_TAG}"