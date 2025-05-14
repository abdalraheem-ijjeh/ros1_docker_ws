# Use ROS Noetic on Ubuntu Focal
FROM ros:noetic-ros-core-focal

ENV DEBIAN_FRONTEND=noninteractive

ENV NODE=image_publisher.py

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-catkin-tools \
      python3-opencv \
      python3-pil \
      exiftool \
      ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
COPY src/ $CATKIN_WS/src/

WORKDIR $CATKIN_WS
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/bin/bash","-c", \
  "source /opt/ros/noetic/setup.bash && \
   source /root/catkin_ws/devel/setup.bash && \
   echo 'ROS_MASTER_URI is' $ROS_MASTER_URI && \
   rosrun sv01_uav_bs_transmission $NODE" \
]

