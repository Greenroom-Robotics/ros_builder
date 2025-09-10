#!/usr/bin/env python3

import argparse
import subprocess
from typing import Dict, List

UBUNTU_VERSION = "24.04"
UBUNTU_CODENAME = "noble"
CUDA_12_6_TRT_CONTAINER_VERSION = "24.11"
CUDA_12_9_TRT_CONTAINER_VERSION = "25.06"

ENV = Dict[str, str]


def build_image(
    base_image: str, ros_distro: str, arch: str, tags: List[str], push: bool = False, env: ENV = {}
):
    print(f"\033[92mBuilding image with base image {base_image} and tags {tags}\033[0m")

    command = [
        "docker buildx build",
        f"--platform linux/{arch}",
        f'--build-arg BASE_IMAGE="{base_image}"',
        f'--build-arg ROS_DISTRO="{ros_distro}"',
        "--provenance=false",
        " ".join([f"-t {tag}" for tag in tags]),
        "--push" if push else "",
        ".",
    ]
    command_str = " ".join(command)
    result = subprocess.run(command_str, shell=True, env=env)
    if result.returncode != 0:
        raise Exception(f"Failed to build image with command {command_str}")

def get_cuda_base_image(arch: str, container_version: str = CUDA_12_6_TRT_CONTAINER_VERSION) -> str:
    base_img = f"nvcr.io/nvidia/tensorrt:{container_version}-py3"
    if arch == "arm64":
        # jetson has an integrated gpu
        base_img += "-igpu"

    return base_img

def build_amd64_specific_images(args):
    # CUDA 12.4 - This requires the cuda base to be built manually.
    build_image(
        base_image="ghcr.io/greenroom-robotics/cuda:12.4",
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda-12.4-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda-12.4-{args.arch}",
        ],
        push=args.push,
    )

    # CUDA 12.9 - not yet supported on jetson/arm
    build_image(
        base_image=get_cuda_base_image(args.arch, container_version=CUDA_12_9_TRT_CONTAINER_VERSION),
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda-12.9-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda-12.9-{args.arch}",
        ],
        push=args.push,
    )


def main():
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ros_distro", required=True, help="ROS2 distro to build (e.g. galactic, humble, etc.)"
    )
    parser.add_argument("--version", required=True, help="Version of the image (e.g. 1.0.0)")
    parser.add_argument(
        "--arch", required=True, help="architecture of the image (e.g. amd64, arm64, etc.)"
    )
    parser.add_argument(
        "--no-cuda", required=False, action="store_true", help="skip building CUDA images"
    )
    parser.add_argument(
        "--push", default=False, type=bool, help="Should we push the image to the registry?"
    )
    args = parser.parse_args()

    # Build images

    # base image (no CUDA)
    build_image(
        base_image=f"ubuntu:{UBUNTU_CODENAME}",
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-{args.arch}",
        ],
        push=args.push,
    )

    if args.no_cuda:
        # return before building CUDA containers if we are not building them
        return

    # CUDA 12.6
    build_image(
        base_image=get_cuda_base_image(args.arch),
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda-12.6-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda-12.6-{args.arch}",
        ],
        push=args.push,
    )

    if args.arch == "amd64":
        build_amd64_specific_images(args)


if __name__ == "__main__":
    main()
