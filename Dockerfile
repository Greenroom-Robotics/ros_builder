ARG BASE_IMAGE

FROM ${BASE_IMAGE}
    
LABEL org.opencontainers.image.source=https://github.com/Greenroom-Robotics/ros_builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=galactic
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
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools and ros2 packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    curl \
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
    ros-${ROS_DISTRO}-ros-base=0.9.3-2* \
    ros-${ROS_DISTRO}-ros-core=0.9.3-2* \
    wget \
    && rm -rf /var/lib/apt/lists/*

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
RUN mkdir /opt/ros/galactic-ext && sudo chown -R ros:ros /opt/ros/galactic-ext

RUN source /opt/ros/galactic/setup.sh && colcon build --base-paths external --merge-install --install-base /opt/ros/galactic-ext --cmake-args -DBUILD_TESTING=OFF -DFASTDDS_STATISTICS=ON
RUN source /opt/ros/galactic-ext/setup.sh && colcon build --base-paths interfaces --merge-install --install-base /opt/ros/galactic-ext --cmake-args -DBUILD_TESTING=OFF

ENV ROS_OVERLAY /opt/ros/galactic-ext
WORKDIR /home/ros

USER ros
