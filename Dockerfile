FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

#NVIDIA ENV
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    ant \
    binutils-dev \
		ca-certificates \
		ccache \
		cmake \
		cppcheck \
		curl \
		git \
		gnupg \
		lsb-release \
		make \
		python3 \
		python3-dev \
    u-boot-tools \
    util-linux \
		valgrind \
		wget \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN apt update

RUN apt install -y cmake

RUN apt install locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

RUN apt install software-properties-common -y

RUN add-apt-repository universe

# ROS KEY
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS INSTALLATION WITH DEPENDENCIES
RUN apt update && apt-get install -y \
    python-is-python3 \
    nano \
    apt-utils \
    python3-pip \
    clang lldb lld wget lsb-release gnupg openssl \
    ros-humble-desktop \
    ros-humble-sdformat-urdf \
    ros-humble-ros-gz-interfaces\
    ros-humble-hardware-interface\
    ros-humble-ros2-control\
    ros-humble-ros2-controllers\
    ros-humble-xacro \
    ros-humble-ign-ros2-control\
    lsb-release wget gnupg/

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install lsb-release wget gnupg
RUN apt-get -y install ignition-fortress
RUN apt -y install ros-humble-ros-gz


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "source /workspaces/hexapod_ws/install/setup.bash" >> ~/.bashrc

RUN echo "alias docker_stop='docker stop hexapod atom'" >> ~/.bashrc

RUN apt update && apt install python3-colcon-common-extensions -y

SHELL ["/bin/bash","-c"]

# Command to run when starting the container
CMD /bin/bash
