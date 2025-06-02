############################
# STAGE 1 – build workspace
############################
FROM ros:noetic-ros-base AS builder
ENV DEBIAN_FRONTEND=noninteractive

# 1) Remove any pre-existing ROS list so we can re-add packages.ros.org
RUN rm -f /etc/apt/sources.list.d/*ros*.list

# 2) Install curl / gnupg2 / lsb-release & enable universe
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      software-properties-common && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# 3) Import the ROS key and point apt at packages.ros.org
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros-latest.list

# 4) Install just enough to run rosdep & catkin
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-rosdep \
      ros-noetic-catkin \
      ros-noetic-std-msgs \
      libimage-exiftool-perl && \
    rm -rf /var/lib/apt/lists/*

# 5) Refresh rosdep database
RUN rosdep update

# 6) Copy your catkin‐ws sources
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}
COPY . ${CATKIN_WS}/src/sv01_uav_bs_transmission

# 7) Let rosdep pull in all declared dependencies
RUN apt-get update && \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
                   rosdep install --rosdistro noetic --from-paths src --ignore-src -y" && \
    rm -rf /var/lib/apt/lists/*

# 8) Build & install into “install” space
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
                   catkin_make -DCMAKE_BUILD_TYPE=Release && \
                   catkin_make install"


############################
# STAGE 2 – runtime image
############################
FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# 1) copy workspace
COPY --from=builder /root/catkin_ws /root/catkin_ws
ENV CATKIN_WS=/root/catkin_ws

# 2) Fix apt sources
# 2a. use HTTPS for Ubuntu mirrors (we already added this)
RUN sed -i 's|http://|https://|g' /etc/apt/sources.list
# 2b. point ROS list at the readonly archive (no key problem there)
RUN sed -i 's|http://packages.ros.org/ros|http://archive.ros.org/ros|g' \
        /etc/apt/sources.list.d/ros*.list || true

# 3) now install the runtime deps
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        libimage-exiftool-perl && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
WORKDIR ${CATKIN_WS}
CMD ["bash", "-lc", "source install/setup.bash && rosnode list"]
