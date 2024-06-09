FROM ros:humble

# Set the working directory
WORKDIR /ws

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    gdb \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-octomap \
    ros-humble-octomap-mapping \
    ros-humble-octomap-msgs \
    ros-humble-octomap-ros \
    ros-humble-octomap-rviz-plugins \
    ros-humble-octomap-server \
    ros-humble-dynamic-edt-3d

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    libompl-dev

RUN apt-get update && apt-get install -y \
    libwebsocketpp-dev \
    nlohmann-json3-dev

RUN apt-get update && apt-get install -y \
    python3-pip
RUN pip3 install matplotlib numpy