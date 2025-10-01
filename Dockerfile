ARG BASE_IMAGE

FROM ${BASE_IMAGE}

ARG ROS_DISTRO

LABEL org.opencontainers.image.source=https://github.com/Greenroom-Robotics/ros_builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO="${ROS_DISTRO}"
ENV ROS_PYTHON_VERSION=3
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# setup debconf for non-interactive install
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
  echo 'wireshark-common wireshark-common/install-setuid boolean true' | debconf-set-selections

# install packages
RUN apt-get update && apt-get dist-upgrade -q -y && apt-get install -q -y --no-install-recommends \
    less \
    iproute2 \
    dirmngr \
    gnupg2 \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# setup ros2 apt sources
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
  apt install /tmp/ros2-apt-source.deb

# setup vulcanexus keys
# RUN curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg \

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# install bootstrap tools and ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    gdb \
    cmake \
    debhelper \
    dh-python \
    dpkg-dev \
    fakeroot \
    git \
    jq \
    iputils-ping \
    python3-catkin-pkg \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-invoke \
    python3-pip \
    python3-pytest-cov \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    clang-format \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-example-interfaces \
    wget \
    bpfcc-tools \
    bpftrace
    # vulcanexus-${ROS_DISTRO}-core \

RUN curl -L https://github.com/rr-debugger/rr/releases/download/5.9.0/rr-5.9.0-Linux-$(uname -m).deb --output rr.deb && dpkg --install rr.deb && rm rr.deb

# Remove EXTERNALLY-MANAGED so we don't need to add --break-system-packages to pip
RUN sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED

# Remove EXTERNALLY-MANAGED so we don't need to add --break-system-packages to pip
RUN sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO --include-eol-distros

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install nodejs
RUN curl -sL https://deb.nodesource.com/setup_22.x | bash -

# install yarn and pyright
RUN apt-get install -y nodejs && npm install --global yarn pyright

RUN pip install pre-commit

# Install Greenroom fork of bloom
RUN pip install https://github.com/Greenroom-Robotics/bloom/archive/refs/heads/gr.zip

# Install Greenroom's rosdep fork which allows installation from URLs, version pinning and downgrades
RUN apt-get remove python3-rosdep -y
RUN pip install -U https://github.com/Greenroom-Robotics/rosdep/archive/refs/heads/greenroom.zip

RUN useradd --create-home --home /home/ros --shell /bin/bash --uid 1000 ros && \
    passwd -d ros && \
    usermod -a -G audio,video,sudo,plugdev,dialout ros

# Build external source packages
WORKDIR /home/ros

# TODO move external repos to packages
COPY ./external.repos ./external.repos
RUN mkdir external
RUN vcs import external < ./external.repos
RUN apt-get update && rosdep update --include-eol-distros && rosdep install -y -i --from-paths external  --include-eol-distros --include-eol-distros 
RUN pip install lark-parser

RUN mkdir /opt/ros/${ROS_DISTRO}-ext && sudo chown -R ros:ros /opt/ros/${ROS_DISTRO}-ext

RUN source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --base-paths external --merge-install --install-base /opt/ros/${ROS_DISTRO}-ext --cmake-args -DBUILD_TESTING=OFF

RUN --mount=type=bind,source=scripts,target=scripts \
  source /opt/ros/${ROS_DISTRO}-ext/setup.sh && python3 scripts/rosidl_generate_inplace.py

ENV ROS_OVERLAY=/opt/ros/${ROS_DISTRO}-ext
WORKDIR /home/ros
ENV PATH="/home/ros/.local/bin:${PATH}"

# Install greenroom public packages
RUN curl -s https://raw.githubusercontent.com/Greenroom-Robotics/public_packages/main/scripts/setup-apt.sh | bash -s

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp

# Enable caching of apt packages: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

USER ros

# Make sure we own the venv directory if it exists
# This is where packages are installed on l4t / jetson
RUN if [ -d /opt/venv ]; then sudo chown -R ros:ros /opt/venv; fi

# Install poetry as ros user
RUN pip install poetry poetry-plugin-export
