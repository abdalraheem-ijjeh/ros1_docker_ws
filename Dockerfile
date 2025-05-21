############################
# STAGE 1 – build workspace
############################
FROM ros:noetic-ros-base AS builder
ENV DEBIAN_FRONTEND=noninteractive

# 1) Build-time packages + rosdep CLI
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-opencv python3-numpy \
        libimage-exiftool-perl python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 2) Refresh rosdep database (base image is already initialised)
RUN rosdep update

# 3) Workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}
COPY . ${CATKIN_WS}/src/sv01_uav_bs_transmission

# 4) **Install package deps – with a fresh APT index**
RUN apt-get update && \
    rosdep install --rosdistro noetic --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*         # clean again to save space

# 5) Build + install
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             catkin_make -DCMAKE_BUILD_TYPE=Release && \
             catkin_make install"

############################
# STAGE 2 – runtime image
############################
FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# 6) Runtime-only deps
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv python3-numpy libimage-exiftool-perl \
    && rm -rf /var/lib/apt/lists/*

# 7) Copy installed workspace
COPY --from=builder /root/catkin_ws /root/catkin_ws
ENV CATKIN_WS=/root/catkin_ws

# 8) Standard ROS entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
WORKDIR ${CATKIN_WS}
CMD ["bash", "-c", "source devel/setup.bash && rosnode list"]
