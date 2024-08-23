FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    binutils-dev \
		cmake \
		curl \
		git \
		python3 \
		python3-dev \
		python3-pip \
		unzip \
		wget \
		zip \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN apt update && apt install locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

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
    ros-humble-desktop \
    ros-humble-sdformat-urdf \
    ros-humble-hardware-interface\
    ros-humble-ros-gz-interfaces\
    ros-humble-ros2-control\
    ros-humble-ros2-controllers\
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ign-ros2-control\
    ros-humble-ros-gz-interfaces\
    lsb-release wget gnupg/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "source /workspaces/hexapod_ws/install/setup.bash" >> ~/.bashrc

RUN echo "alias docker_stop='docker stop hexapod atom'" >> ~/.bashrc

RUN apt update && apt install python3-colcon-common-extensions -y

RUN pip3 install adafruit-circuitpython-servokit

RUN apt-get install python3-dev python3-rpi.gpio
RUN apt-get install RPi.GPIO
SHELL ["/bin/bash","-c"]

# Command to run when starting the container
CMD /bin/bash
