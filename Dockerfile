FROM ros:galactic-ros-base
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


RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    dpkg-dev \
    debhelper \
    dh-python \
    fakeroot \
    git \
    jq \
    python3-catkin-pkg \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sL https://deb.nodesource.com/setup_14.x | bash -
# install yarn and pyright
RUN apt-get install -y nodejs && npm install --global yarn pyright

# add any custom commands required for building TODO replace with debian package
ADD tools/* /usr/bin/

# Install Greenroom fork of bloom
RUN pip install https://github.com/Greenroom-Robotics/bloom/archive/refs/heads/david_revay/sc-4323/version-pinning-in-package-xml.zip

# Install Greenroom's rosdep fork which does not check if packages are installed correctly.
# this allows us to add paths to python packages stored in github where the path != package_name
RUN apt-get remove python3-rosdep -y
RUN pip install https://github.com/Greenroom-Robotics/rosdep/archive/refs/heads/david_revay/sc-4323/version-pinning-in-package-xml.zip

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
RUN source /opt/ros/galactic/setup.sh && colcon build --base-paths external --merge-install --install-base /opt/ros/galactic --cmake-args -DBUILD_TESTING=OFF -DFASTDDS_STATISTICS=ON
RUN source /opt/ros/galactic/setup.sh && colcon build --base-paths interfaces --merge-install --install-base /opt/ros/galactic --cmake-args -DBUILD_TESTING=OFF

WORKDIR /home/ros
USER ros
