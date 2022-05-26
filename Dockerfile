FROM ros:galactic-ros-base
LABEL org.opencontainers.image.source=https://github.com/Greenroom-Robotics/ros_builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=galactic
ENV ROS_PYTHON_VERSION=3
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

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
    libopus-dev \
    libvpx-dev \
    python3-bloom \
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

# Note, we install libopus-dev and libvpx-dev because aiortc needs it
# This is not ideal....

# add any custom commands required for building
ADD tools/* /usr/bin/

# Install Greenroom fork of bloom
RUN pip install https://github.com/Greenroom-Robotics/bloom/archive/refs/heads/feature/fix-namespace.zip

# Install Greenroom's rosdep fork which does not check if packages are installed correctly.
# this allows us to add paths to python packages stored in github where the path != package_name
RUN dpkg -r --force-depends "python3-rosdep"
RUN pip install https://github.com/Greenroom-Robotics/rosdep/archive/1f560a73553e6e8d262cf0be19b6b384be90fbd2.zip

RUN useradd --create-home --home /home/ros --shell /bin/bash --uid 1000 ros && \
    passwd -d ros && \
    usermod -a -G audio,video,sudo,plugdev,dialout ros

# Build external deps
WORKDIR /home/ros
COPY ./external.repos ./external.repos
RUN mkdir external
RUN vcs import external < ./external.repos
RUN source /opt/ros/galactic/setup.sh && colcon build --merge-install --install-base /opt/ros/galactic --cmake-args -DBUILD_TESTING=OFF

RUN mkdir /opt/greenroom && chown ros:ros /opt/greenroom

WORKDIR /home/ros
USER ros
