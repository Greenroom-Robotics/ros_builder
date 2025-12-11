#!/usr/bin/env python3

import argparse
import subprocess

UBUNTU_CODENAME = "noble"
DEEPSTREAM_VERSION = "8.0"
GHCR_REGISTRY = "ghcr.io/greenroom-robotics"


def build_image(
    arch: str,
    tags: list[str],
    push: bool = False,
    no_cache: bool = False,
    build_args: dict[str, str] = {},
) -> None:
    command = [
        "docker buildx build",
        f"--platform linux/{arch}",
    ]

    for key, value in build_args.items():
        command.append(f'--build-arg {key}="{value}"')

    command.extend(
        [
            "--provenance=false",
            " ".join([f"-t {tag}" for tag in tags]),
            "--no-cache" if no_cache else "",
            "--push" if push else "",
            ".",
        ]
    )

    command_str = " ".join(command)
    print(f"Docker build command: {command_str}")
    result = subprocess.run(command_str, shell=True)
    if result.returncode != 0:
        raise Exception(f"Failed to build image with command {command_str}")


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
    parser.add_argument("--push", action="store_true", help="Push the images to the registry")
    parser.add_argument(
        "--no-cache",
        required=False,
        action="store_true",
        help="Build without using cache",
    )
    args = parser.parse_args()

    if not args.no_gpu:
        print("Building ROS Builder base image for GPU")

        # Use pre-squashed DeepStream image from ghcr.io (multi-arch manifest)
        # This must be created first via the squash-deepstream workflow
        # Otherwise, the deepstream image has ~160 layers which causes issues in dependent builds
        squashed_base = f"{GHCR_REGISTRY}/deepstream-squashed:{DEEPSTREAM_VERSION}-triton-multiarch"

        build_image(
            build_args={
                "BASE_IMAGE": squashed_base,
                "BASE_USER": "triton-server",
                "ROS_DISTRO": args.ros_distro,
                "GPU": "true",
            },
            arch=args.arch,
            tags=[
                f"{GHCR_REGISTRY}/ros_builder:{args.ros_distro}-{args.version}-deepstream-{DEEPSTREAM_VERSION}-{args.arch}",
                f"{GHCR_REGISTRY}/ros_builder:{args.ros_distro}-latest-deepstream-{DEEPSTREAM_VERSION}-{args.arch}",
            ],
            push=args.push,
            no_cache=args.no_cache,
        )


    print("Building ROS Builder base image for CPU")
    build_image(
        build_args={
            "BASE_IMAGE": f"ubuntu:{UBUNTU_CODENAME}",
            "BASE_USER": "ubuntu",
            "ROS_DISTRO": args.ros_distro,
            "GPU": "false",
        },
        arch=args.arch,
        tags=[
            f"{GHCR_REGISTRY}/ros_builder:{args.ros_distro}-{args.version}-{args.arch}",
            f"{GHCR_REGISTRY}/ros_builder:{args.ros_distro}-latest-{args.arch}",
        ],
        push=args.push,
        no_cache=args.no_cache,
    )

if __name__ == "__main__":
    main()
