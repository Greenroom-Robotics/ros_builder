ARG BASE_IMAGE

FROM ${BASE_IMAGE}

ARG ROS_DISTRO

LABEL org.opencontainers.image.source=https://github.com/Greenroom-Robotics/ros_builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO="${ROS_DISTRO}"
ENV ROS_PYTHON_VERSION=3
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp
ENV FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;\
SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC;\
HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;\
GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;SAMPLE_DATAS_TOPIC;\
PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC"


# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# setup ros2 apt sources
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/vulcanexus.list

# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools and ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    gcc-12-base \
    g++-12 \
    cmake \
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
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros-core \
    vulcanexus-${ROS_DISTRO}-core \
    wget

RUN curl -L https://github.com/mozilla/sccache/releases/download/v0.4.0/sccache-v0.4.0-$(uname -m)-unknown-linux-musl.tar.gz | tar zx --wildcards "*/sccache" --strip-components 1 --directory=/usr/bin

# set gcc version to latest available on ubuntu rel
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 12 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 12 && \
    update-alternatives --install /usr/bin/gcc-ar gcc-ar /usr/bin/gcc-ar-12 12 && \
    update-alternatives --install /usr/bin/gcc-nm gcc-nm /usr/bin/gcc-nm-12 12 && \
    update-alternatives --install /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-12 12

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
RUN curl -sL https://deb.nodesource.com/setup_18.x | bash -

# install yarn and pyright
RUN apt-get install -y nodejs && npm install --global yarn pyright

RUN pip install pre-commit

# Install Greenroom fork of bloom
RUN pip install https://github.com/Greenroom-Robotics/bloom/archive/refs/heads/gr.zip

# Install Greenroom's rosdep fork which allows installation from URLs and specific versions
RUN apt-get remove python3-rosdep -y
RUN pip install -U https://github.com/Greenroom-Robotics/rosdep/archive/refs/heads/greenroom.zip

RUN useradd --create-home --home /home/ros --shell /bin/bash --uid 1000 ros && \
    passwd -d ros && \
    usermod -a -G audio,video,sudo,plugdev,dialout ros

# Build external source packages
WORKDIR /home/ros
COPY ./external.repos ./external.repos
COPY ./interfaces.repos ./interfaces.repos
RUN mkdir external && mkdir interfaces
RUN vcs import external < ./external.repos
RUN vcs import interfaces < ./interfaces.repos
RUN apt-get update && rosdep update && rosdep install -y -i --from-paths external

# Install external first to ensure interfaces are built correctly
RUN mkdir /opt/ros/${ROS_DISTRO}-ext && sudo chown -R ros:ros /opt/ros/${ROS_DISTRO}-ext

RUN source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --base-paths external --merge-install --install-base /opt/ros/${ROS_DISTRO}-ext --cmake-args -DBUILD_TESTING=OFF
# TODO remove interface building and use rosidl generate mypy typing
RUN source /opt/ros/${ROS_DISTRO}-ext/setup.sh && colcon build --base-paths interfaces --merge-install --install-base /opt/ros/${ROS_DISTRO}-ext --cmake-args -DBUILD_TESTING=OFF

ENV ROS_OVERLAY /opt/ros/${ROS_DISTRO}-ext
WORKDIR /home/ros
ENV PATH="/home/ros/.local/bin:${PATH}"

# Enable caching of apt packages: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

USER ros

# Install poetry as ros user
RUN curl -sSL https://install.python-poetry.org | python3 -
