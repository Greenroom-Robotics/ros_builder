#!/usr/bin/env python3

import argparse
import subprocess

UBUNTU_VERSION = "24.04"
UBUNTU_CODENAME = "noble"
DEEPSTREAM_VERSION = "8.0"

ENV = dict[str, str]


def build_image(
    base_image: str,
    ros_distro: str,
    base_user: str,
    arch: str,
    tags: list[str],
    push: bool = False,
    env: ENV = {},
):
    print(f"\033[92mBuilding image with base image {base_image} and tags {tags}\033[0m")

    command = [
        "docker buildx build",
        f"--platform linux/{arch}",
        f'--build-arg BASE_IMAGE="{base_image}"',
        f'--build-arg ROS_DISTRO="{ros_distro}"',
        f'--build-arg BASE_IMAGE_USER="{base_user}"',
        "--provenance=false",
        " ".join([f"-t {tag}" for tag in tags]),
        "--push" if push else "",
        ".",
    ]
    command_str = " ".join(command)
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
        "--no-gpu",
        required=False,
        action="store_true",
        help="Skip building GPU based images (contain deepstream/tensorrt/CUDA)",
    )
    parser.add_argument("--push", default=False, type=bool, help="Push the images to the registry")
    args = parser.parse_args()

    # Build non GPU base image
    build_image(
        base_image=f"ubuntu:{UBUNTU_CODENAME}",
        ros_distro=args.ros_distro,
        base_user="ubuntu",
        arch=args.arch,
        tags=[
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-{args.arch}",
            f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-{args.arch}",
        ],
        push=args.push,
    )

    if not args.no_gpu:
        # Build Deepstream base image
        DEEPSTREAM_VERSION = "8.0"
        build_image(
            base_image=f"nvcr.io/nvidia/deepstream:{DEEPSTREAM_VERSION}-triton-multiarch",
            ros_distro=args.ros_distro,
            base_user="triton-server",
            arch=args.arch,
            tags=[
                f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-ds-{DEEPSTREAM_VERSION}-{args.arch}",
                f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-ds-{DEEPSTREAM_VERSION}-{args.arch}",
            ],
            push=args.push,
        )


if __name__ == "__main__":
    main()
