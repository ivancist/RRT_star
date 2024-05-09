FROM ros:humble

# Set the working directory
WORKDIR /ws

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    gdb \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-octomap \
    ros-humble-octomap-msgs

# Development tools (rviz2)
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-octomap-rviz-plugins

#RUN apt-get update && apt-get install -y \
#     cmake \
#    doxygen
#
## Clone the Open3D repository
#RUN git clone https://github.com/isl-org/Open3D.git
#
## Install dependencies
#RUN yes | ./Open3D/util/install_deps_ubuntu.sh
#
## Create a build directory and navigate into it
#RUN mkdir Open3D/build && cd Open3D/build && cmake -DPYTHON_EXECUTABLE=`which python` ..
##RUN cd Open3D/build && make -j$(nproc)
#RUN ln -s /usr/bin/python3 /usr/bin/python && cd Open3D/build \
#    && make -j$(nproc) || true && make -j$(nproc) || true && make -j$(nproc) || true && make -j$(nproc) || true && make -j$(nproc) || true && make -j$(nproc) || true && make -j$(nproc) || true && sed -i '40i #undef M_PIf' /ws/Open3D/build/filament/src/ext_filament/libs/image/src/ImageSampler.cpp && make -j$(nproc)
#RUN cd Open3D/build && make install
#
#RUN apt-get install nano
#
#RUN apt-get update && apt-get install --no-install-recommends -y \
#    libegl1 \
#    libgl1 \
#    libgomp1 \
#    python3-pip \
#    && rm -rf /var/lib/apt/lists/*
#
## Install Open3D from the PyPI repositories
#RUN python3 -m pip install --no-cache-dir --upgrade pip && \
#    python3 -m pip install --no-cache-dir --upgrade open3d
