#!/bin/python3

import argparse
import subprocess
from typing import List, Dict


ENV = Dict[str, str]


def build_image(base_image: str, ros_distro: str, arch: str, tags: List[str], push: bool = False, env: ENV = {}):
    print(
        f"\033[92mBuilding image with base image {base_image} and tags {tags}\033[0m")

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


if __name__ == "__main__":
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros_distro', required=True,
                        help="ROS2 distro to build (e.g. galactic, humble, etc.)")
    parser.add_argument('--version', required=True,
                        help="Version of the image (e.g. 1.0.0)")
    parser.add_argument('--arch', required=True,
                        help="architecture of the image (e.g. amd64, arm64, etc.)")
    parser.add_argument('--push', default=False, type=bool,
                        help="Should we push the image to the registry?")
    args = parser.parse_args()


    if args.ros_distro == "jazzy":
        UBUNTU_VERSION = "24.04"
        UBUNTU_CODENAME = "noble"
        CUDA_VERSION = f"12.6.3-cudnn-devel-ubuntu{UBUNTU_VERSION}"
    elif args.ros_distro == "iron":
        UBUNTU_VERSION = "22.04"
        UBUNTU_CODENAME = "jammy"
        CUDA_VERSION = f"12.6.3-cudnn-devel-ubuntu{UBUNTU_VERSION}"

    # Build images
    build_image(
        base_image=f"nvidia/cuda:{CUDA_VERSION}",
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda-{args.arch}"
        ],
        push=args.push,
    )
    build_image(
        base_image=f"ubuntu:{UBUNTU_CODENAME}",
        ros_distro=args.ros_distro,
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-{args.arch}"
        ],
        push=args.push,
    )
