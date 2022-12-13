#!/bin/python3

import argparse
import subprocess
from typing import List, Dict


UBUNTU_VERSION = "22.04"
UBUNTU_CODENAME = "jammy"
CUDA_VERSION = "11.7.0-devel-ubuntu22.04"

ENV = Dict[str, str]


def build_image(base_image: str, ros_distro: str, tags: List[str], push: bool = False, env: ENV = {}):
    print(
        f"\033[92mBuilding image with base image {base_image} and tags {tags}\033[0m")

    command = [
        "docker buildx build",
        "--platform linux/amd64,linux/arm64",
        f'--build-arg BASE_IMAGE="{base_image}"',
        f'--build-arg ROS_DISTRO="{ros_distro}"',
        " ".join([f"-t {tag}" for tag in tags]),
        "--push" if push else "",
        ".",
    ]
    command_str = " ".join(command)
    subprocess.run(command_str, shell=True, env=env)


if __name__ == "__main__":
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros_distro', required=True,
                        help="ROS distro to build (e.g. galactic, humble, etc.)")
    parser.add_argument('--version', required=True,
                        help="Version of the image (e.g. 1.0.0)")
    parser.add_argument('--push', default=False,
                        help="Should we push the image to the registry?")
    args = parser.parse_args()

    # Make sure buildkit is enabled
    env: ENV = {"DOCKER_BUILDKIT": "1"}

    # Run binfmt
    subprocess.run(
        "docker run --privileged --rm tonistiigi/binfmt --install all", shell=True, env=env)

    # Create buildx environment
    subprocess.run(
        "docker buildx create --name ros_builder --use || echo 'ros_builder buildx already exists. Continuing...'", shell=True, env=env)

    # Build images
    build_image(
        base_image=f"nvidia/cuda:{CUDA_VERSION}",
        ros_distro=args.ros_distro,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest"
        ],
        push=args.push,
        env=env,
    )
    build_image(
        base_image=f"ubuntu:{UBUNTU_CODENAME}",
        ros_distro=args.ros_distro,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-cuda",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-cuda"
        ],
        push=args.push,
        env=env,
    )