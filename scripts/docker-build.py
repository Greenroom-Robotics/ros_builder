#!/usr/bin/env python3

import argparse
import os
import subprocess

UBUNTU_CODENAME = "noble"
DEEPSTREAM_VERSION = "8.0"

ENV = dict[str, str]


def build(
    arch: str,
    tags: list[str],
    push: bool = False,
    dockerfile: str = "Dockerfile",
    build_args: dict[str, str] = {},
    env: ENV = {},
) -> None:
    command = [
        "docker buildx build",
        f"--platform linux/{arch}",
        f"-f {dockerfile}",
    ]

    for key, value in build_args.items():
        command.append(f'--build-arg {key}="{value}"')

    command.extend(
        [
            "--provenance=false",
            " ".join([f"-t {tag}" for tag in tags]),
            "--push" if push else "",
            ".",
        ]
    )

    command_str = " ".join(command)
    print(f"Docker build command: {command_str}")
    result = subprocess.run(command_str, shell=True, env=env if env else None)
    if result.returncode != 0:
        raise Exception(f"Failed to build image with command {command_str}")


def download_dependency(name: str, url: str) -> None:
    os.makedirs("deps", exist_ok=True)
    filepath = os.path.join("deps", name)

    print(f"Downloading: {url}")
    result = subprocess.run(
        f"wget -nc --content-disposition '{url}' --output-document {name}",
        shell=True,
        cwd="deps",
    )

    if result.returncode != 0:
        if result.returncode == 1 and os.path.exists(filepath):
            print(f"File {name} already exists, skipping download")
        else:
            raise Exception(f"Failed to download {url}")


def build_deepstream_image(
    arch: str,
    tags: list[str],
    push: bool = False,
) -> None:
    print("Building DeepStream base image")
    download_dependency(
        name="deepstream-8.0_8.0.0-1_amd64.deb",
        url="https://api.ngc.nvidia.com/v2/resources/org/nvidia/deepstream/8.0/files?redirect=true&path=deepstream-8.0_8.0.0-1_amd64.deb",
    )
    build(
        arch=arch,
        tags=tags,
        push=push,
        dockerfile="Dockerfile.deepstream",
    )


def build_ros_image(
    base_image: str,
    ros_distro: str,
    base_user: str,
    arch: str,
    tags: list[str],
    push: bool = False,
    env: ENV = {},
):
    print("Building ROS Builder base image")
    build(
        arch=arch,
        tags=tags,
        push=push,
        build_args={
            "BASE_IMAGE": base_image,
            "ROS_DISTRO": ros_distro,
            "BASE_IMAGE_USER": base_user,
        },
        env=env,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ros_distro", required=True, help="ROS2 distro to build (e.g. galactic, humble, etc.)"
    )
    parser.add_argument("--version", required=True, help="Version of the image (e.g. 1.0.0)")
    parser.add_argument(
        "--arch", required=True, help="Architecture of the image (e.g. amd64, arm64, etc.)"
    )
    parser.add_argument(
        "--no-gpu",
        required=False,
        action="store_true",
        help="Skip building GPU based images (containing deepstream/tensorrt/CUDA)",
    )
    parser.add_argument("--push", default=False, type=bool, help="Push the images to the registry")
    args = parser.parse_args()

    # Build non GPU ros builder base image
    build_ros_image(
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

    # Build GPU ros builder image.
    if not args.no_gpu:
        # Build custom DeepStream base image (prebuilt nvcr.io deepstream images are larger).
        deepstream_base_tag = f"ghcr.io/greenroom-robotics/deepstream:{DEEPSTREAM_VERSION}"
        build_deepstream_image(
            arch=args.arch,
            tags=[deepstream_base_tag],
            push=args.push,
        )
        build_ros_image(
            base_image=deepstream_base_tag,
            ros_distro=args.ros_distro,
            base_user="ubuntu",
            arch=args.arch,
            tags=[
                f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-{args.version}-ds-{DEEPSTREAM_VERSION}-{args.arch}",
                f"ghcr.io/greenroom-robotics/ros_builder:{args.ros_distro}-latest-ds-{DEEPSTREAM_VERSION}-{args.arch}",
            ],
            push=args.push,
        )


if __name__ == "__main__":
    main()
