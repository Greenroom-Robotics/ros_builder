#!/usr/bin/env python3

import argparse
import subprocess
from typing import Dict, List

UBUNTU_VERSION = "24.04"
UBUNTU_CODENAME = "noble"
CUDA_VERSION = f"12.6.3-cudnn-devel-ubuntu{UBUNTU_VERSION}"
TRT_CONTAINER_VERSION = "24.11"

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
    print(f"building with: {command_str}")
    result = subprocess.run(command_str, shell=True, env=env)
    if result.returncode != 0:
        raise Exception(f"Failed to build image with command {command_str}")


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
        return

    base_cuda_img = f"nvcr.io/nvidia/tensorrt:{TRT_CONTAINER_VERSION}-py3"
    if args.arch == "arm64":
        # jetson has an integrated gpu
        base_cuda_img += "-igpu"

    build_image(
        base_image=base_cuda_img,
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda-12.6-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda-12.6-{args.arch}",
        ],
        push=args.push,
    )

    if args.arch == "amd64":
        # 12.4 for x86 - This requires the cuda base to be built manually.
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


if __name__ == "__main__":
    main()
