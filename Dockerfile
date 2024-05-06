ARG BASE_IMAGE

FROM ${BASE_IMAGE}

ARG ROS_DISTRO

LABEL org.opencontainers.image.source=https://github.com/Greenroom-Robotics/ros_builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO="${ROS_DISTRO}"
ENV ROS_PYTHON_VERSION=3
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# setup debconf for non-interactive install
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
  echo 'wireshark-common wireshark-common/install-setuid boolean true' | debconf-set-selections

# install packages
RUN apt-get update && apt-get dist-upgrade -q -y && apt-get install -q -y --no-install-recommends \
    less \
    dirmngr \
    gnupg2 \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# setup ros2 apt sources
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/vulcanexus.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-stable-keyring.gpg]  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/gazebo-stable.list

# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-stable-keyring.gpg
# && curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg \

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools and ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    gcc-14-base \
    g++-14 \
    gdb \
    rr \
    cmake \
    sccache \
    debhelper \
    dh-python \
    dpkg-dev \
    fakeroot \
    git \
    jq \
    python3-catkin-pkg \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-invoke \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    clang-format \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-example-interfaces \
    wget

    # vulcanexus-${ROS_DISTRO}-core \

# RUN curl -L https://github.com/mozilla/sccache/releases/download/v0.7.7/sccache-v0.7.7-$(uname -m)-unknown-linux-musl.tar.gz | tar zx --wildcards "*/sccache" --strip-components 1 --directory=/usr/bin
# RUN curl -L https://github.com/rr-debugger/rr/releases/download/5.7.0/rr-5.7.0-Linux-$(uname -m).deb --output rr.deb && dpkg --install rr.deb && rm rr.deb

# set gcc version to latest available on ubuntu rel
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 14 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 14 && \
    update-alternatives --install /usr/bin/gcc-ar gcc-ar /usr/bin/gcc-ar-14 14 && \
    update-alternatives --install /usr/bin/gcc-nm gcc-nm /usr/bin/gcc-nm-14 14 && \
    update-alternatives --install /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-14 14

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install nodejs
RUN curl -sL https://deb.nodesource.com/setup_20.x | bash -

# install yarn and pyright
RUN apt-get install -y nodejs && npm install --global yarn pyright

RUN pip install --break-system-packages pre-commit

# Install Greenroom fork of bloom
RUN pip install --break-system-packages https://github.com/Greenroom-Robotics/bloom/archive/refs/heads/gr.zip

# Install Greenroom's rosdep fork which allows installation from URLs and specific versions with downgrades
RUN apt-get remove python3-rosdep -y
RUN pip install --break-system-packages -U https://github.com/Greenroom-Robotics/rosdep/archive/refs/heads/greenroom.zip

RUN usermod --move-home --home /home/ros --login ros ubuntu && \
    usermod -a -G audio,video,sudo,plugdev,dialout ros && \
    passwd -d ros && \
    groupmod --new-name ros ubuntu

# Build external source packages
WORKDIR /home/ros

# TODO move external repos to packages
COPY ./external.repos ./external.repos
RUN mkdir external
RUN vcs import external < ./external.repos
RUN apt-get update && rosdep update && rosdep install -y -i --from-paths external

RUN mkdir /opt/ros/${ROS_DISTRO}-ext && sudo chown -R ros:ros /opt/ros/${ROS_DISTRO}-ext

RUN source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --base-paths external --merge-install --install-base /opt/ros/${ROS_DISTRO}-ext --cmake-args -DBUILD_TESTING=OFF

RUN --mount=type=bind,source=scripts,target=scripts \
  source /opt/ros/${ROS_DISTRO}-ext/setup.sh && python3 scripts/rosidl_generate_inplace.py

ENV ROS_OVERLAY /opt/ros/${ROS_DISTRO}-ext
WORKDIR /home/ros
ENV PATH="/home/ros/.local/bin:${PATH}"

# Enable caching of apt packages: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

USER ros

# Install poetry as ros user
# RUN curl -sSL https://install.python-poetry.org | python3 -
RUN pip install --break-system-packages poetry
