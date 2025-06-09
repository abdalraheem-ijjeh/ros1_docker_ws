# Multi‑Drone Image Pipeline (ROS Noetic)

This repository provides `sv01_uav_bs_transmission`, a ROS Noetic package plus a
Docker‑based workflow that lets any number of drones publish JPEG/PNG frames
(with full EXIF metadata) to a base‑station that files every image under

```
<output_root>/<drone_id>/flight_<n>/IMG_0001.jpg
```

The system works unchanged on a single laptop (all nodes in containers) or
across multiple machines (one master, N publishers, one subscriber).

---

## 1 Quick architecture recap

```
[drones]  image_publisher.py  ──►  /image_meta  ──►  image_subscriber.py [base]
                 (docker)               │                    (docker)
           ── sensor_msgs/CompressedImage + key/value metadata ──
```

* `drone_id` + `flight_num` are carried inside the custom message.
* All EXIF tags are preserved loss‑less via ExifTool JSON.

---

## 2  Prerequisites

| Host role | You need                     |
|-----------|------------------------------|
| Any Linux/WSL host | Docker 20.10+, `curl`, `awk` |
| Bare‑metal ROS nodes (optional) | `ros-noetic-ros-core`        |

> Ubuntu 22.04 hosts work fine even though the containers run Ubuntu 20.04.

---

## 3  Build the image once

```bash
git clone <this repo>
cd sv01_uav_bs_transmission
docker build -t uav_bs:noetic .
```

---

## 4  Single‑laptop demo (all containers)

```bash
export HOST_IP=$(hostname -I | awk '{print $1}')

# 1. Master
docker run -d --rm --net=host --name ros_master   -e ROS_MASTER_URI=http://$HOST_IP:11311 -e ROS_IP=$HOST_IP   ros:noetic-ros-core roscore

# 2. Subscriber
mkdir -p ~/ros1_img_dst
docker run --rm --net=host   -e ROS_MASTER_URI=http://$HOST_IP:11311 -e ROS_IP=$HOST_IP   -v ~/ros1_img_dst:/data   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash &&            rosrun sv01_uav_bs_transmission image_subscriber.py _output_folder:=/data"

# 3. Publisher (use any folder with test images)
docker run --rm --net=host   -e ROS_MASTER_URI=http://$HOST_IP:11311 -e ROS_IP=$HOST_IP   -v ~/test_images:/root/catkin_ws/images:ro   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash &&            rosrun sv01_uav_bs_transmission image_publisher.py            _drone_id:=UAV_01 _flight_num:=1"
```



```bash
export ROS_MASTER_URI=http://192.168.1.45:11311       
export ROS_IP=192.168.1.11                            
```

```bash
# 2 – source ROS & your workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

```bash
# 3 – run the node
rosrun sv01_uav_bs_transmission image_publisher.py \
       _drone_id:=UAV_02 \
       _flight_num:=01 \
       _image_folder:=/home/khadas/programming/Payload-SDK/build/bin
```

```bash
rosrun sv01_uav_bs_transmission image_publisher.py \
       _drone_id:=UAV_02 \
       _flight_num:=01 \
       _image_folder:=/home/khadas/programming/Onboard-SDK/build/bin
```



1. ROS master (ROSCORE)

docker run --rm -d --net=host --name ros_master   -e ROS_MASTER_URI=http://192.168.1.45:11311   -e ROS_IP=192.168.1.45   ros:noetic-ros-core   roscore

2. subscriber

docker run --rm --net=host   -e ROS_MASTER_URI=http://192.168.1.45:11311   -e ROS_IP=192.168.1.45   -v /mnt/storage/home/abdalraheem/Documents/live_trials/Seville_GRVC/missions/mission_data/:/data   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_subscriber.py _output_folder:=/data"

docker run --rm --net=host   -e ROS_MASTER_URI=http://127.0.0.1:11311   -e ROS_IP=127.0.0.1   -v /mnt/storage/home/abdalraheem/Documents/live_trials/Sevilla_Utrera/missions/mission_data/:/data   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_subscriber.py _output_folder:=/data"

3. publisher (UAV_01)
docker run --rm --net=host   -e ROS_MASTER_URI=http://$HOST_IP:11311   -e ROS_IP=$HOST_IP   -v /home/grvc/ros1_images:/images:ro   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash && \
rosrun sv01_uav_bs_transmission image_publisher.py \
_drone_id:=UAV_01 \
_flight_num:=02 \
_image_folder:=/images"

4. Publisher (UAV_02)

docker run --rm --net=host   -e ROS_MASTER_URI=http://$HOST_IP:11311   -e ROS_IP=$HOST_IP   -v /home/grvc/ros1_images:/images:ro   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash && \
rosrun sv01_uav_bs_transmission image_publisher.py \
_drone_id:=UAV_02 \
_flight_num:=02 \
_image_folder:=/images"

5. Publisher (UAV03)
docker run --rm --net=host   -e ROS_MASTER_URI=http://$HOST_IP:11311   -e ROS_IP=$HOST_IP   -v /home/grvc/ros1_images:/images:ro   uav_bs:noetic   bash -c "source /root/catkin_ws/devel/setup.bash && \
rosrun sv01_uav_bs_transmission image_publisher.py \
_drone_id:=UAV_03 \
_flight_num:=02 \
_image_folder:=/images"

# #############################################################################
# Modified version
# #############################################################################
1. ROS master (ROSCORE)

docker run --rm -d --net=host --name ros_master   -e ROS_MASTER_URI=http://192.168.1.45:11311   -e ROS_IP=192.168.1.45   ros:noetic-ros-core   roscore

docker run --rm -d --net=host --name ros_master   -e ROS_MASTER_URI=http://127.0.0.1:11311   -e ROS_IP=192.168.1.45   ros:noetic-ros-core   roscore


2. subscriber

docker run --rm --net=host   -e ROS_MASTER_URI=http://192.168.1.45:11311   -e ROS_IP=192.168.1.45   -v /mnt/storage/home/abdalraheem/Documents/live_trials/La_Saeta/missions/mission_1/mission_data/:/data   my_ros_image:latest   bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_subscriber.py _output_folder:=/data"

docker run --rm --net=host \
  -e ROS_MASTER_URI=http://$HOST_IP:11311 \
  -e ROS_IP=$HOST_IP \
  -v /mnt/storage/home/abdalraheem/Documents/live_trials/La_Saeta/missions/mission_1/mission_data/:/data:rw \
  my_ros_image:latest \
  bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_subscriber.py \
             _output_folder:=/data \
             _upscale_factor:=4.0   \
             _queue_size:=500"   # optional – default is 500


3. publisher (UAV_01)
docker run --rm --net=host \
  -e ROS_MASTER_URI=http://$HOST_IP:11311 \
  -e ROS_IP=$HOST_IP \
  -v /home/grvc/ros1_images:/images:ro \
  my_ros_image:latest \
  bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_publisher.py \
             _drone_id:=UAV_01 \
             _flight_num:=01 \
             _image_folder:=/images \
             _scale_factor:=0.25 \
             _jpeg_quality:=80"

4. Publisher (UAV_02)

docker run --rm --net=host \
  -e ROS_MASTER_URI=http://$HOST_IP:11311 \
  -e ROS_IP=$HOST_IP \
  -v /home/grvc/ros1_images:/images:ro \
  my_ros_image:latest \
  bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_publisher.py \
             _drone_id:=UAV_02 \
             _flight_num:=01 \
             _image_folder:=/images \
             _scale_factor:=0.25 \
             _jpeg_quality:=80"

5. Publisher (UAV03)
docker run --rm --net=host \
  -e ROS_MASTER_URI=http://$HOST_IP:11311 \
  -e ROS_IP=$HOST_IP \
  -v /home/grvc/ros1_images:/images:ro \
  my_ros_image:latest \
  bash -c "source /root/catkin_ws/devel/setup.bash && \
           rosrun sv01_uav_bs_transmission image_publisher.py \
             _drone_id:=UAV_03 \
             _flight_num:=01 \
             _image_folder:=/images \
             _scale_factor:=0.25 \
             _jpeg_quality:=80"

