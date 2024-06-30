FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

#NVIDIA ENV
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

ENV GZ_VERSION=harmonic

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    ant \
    binutils-dev \
		ca-certificates \
		ccache \
		cmake \
		cppcheck \
		curl \
		dirmngr \
		doxygen \
		g++-multilib \
		gcc-multilib \
		gdb \
    gettext \
		git \
		gnupg \
		gosu \
		lcov \
    libelf-dev \
		libexpat-dev \
    libvecmath-java \
		libfreetype6-dev \
		libgmp-dev \
		libgtest-dev \
		libisl-dev \
		libmpc-dev \
		libmpfr-dev \
		libpng-dev \
		libssl-dev \
		lsb-release \
		make \
		ninja-build \
		openssh-client \
    openjdk-11-jre \
    openjdk-11-jdk \
		python3 \
		python3-dev \
		python3-pip \
		rsync \
    screen \
		shellcheck \
		tzdata \
    texinfo \
    u-boot-tools \
    util-linux \
		unzip \
		valgrind \
		wget \
		xsltproc \
		zip \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN apt update

RUN apt install -y cmake

RUN apt update && apt install locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

RUN apt install software-properties-common -y

RUN add-apt-repository universe

# Install vcpkg dependencies
RUN apt install -y \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev \
    libfftw3-dev \
    libcgal-dev \
    python3

# ROS KEY
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# GZ-SIM KEY and GZ-SIM8 Install
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
        gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ROS INSTALLATION WITH DEPENDENCIES
RUN apt update && apt-get install -y \
    python-is-python3 \
    nano \
    less \
    apt-utils \
    python3-pip \
    nlohmann-json3-dev \
    git cmake python3-vcstools curl \
    clang lldb lld wget lsb-release gnupg openssl \
    ros-humble-desktop \
    ros-humble-sdformat-urdf \
    ros-humble-hardware-interface\
    ros-humble-ros2-control\
    ros-humble-ros2-controllers\
    libgflags-dev \
    ros-humble-xacro \
    tmux \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-gzharmonic\
    #ADD ROS2 CONTROL 
    ros-humble-ign-ros2-control\
    lsb-release wget gnupg/

RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install gz-harmonic
RUN sudo apt-get update
RUN sudo apt-get install lsb-release wget gnupg
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install ignition-fortress


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "source /workspaces/hexapod_ws/install/setup.bash" >> ~/.bashrc

RUN echo "alias docker_stop='docker stop hexapod atom'" >> ~/.bashrc

RUN apt update && apt install python3-colcon-common-extensions -y

SHELL ["/bin/bash","-c"]

# Command to run when starting the container
CMD /bin/bash