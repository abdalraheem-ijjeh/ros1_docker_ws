# sv01_uav_bs_transmission

This package provides a ROS 1 publisher–subscriber system for transmitting images with metadata. The **image_publisher** watches a folder for new images, extracts EXIF and file metadata, and publishes them as custom `ImageWithMetadataComp` messages. The **image_subscriber** receives these messages, embeds the metadata back into the image files using ExifTool, and saves them.

---

## Prerequisites

- **Docker** installed on your machine (no native ROS 1 installation needed).
- A folder of source images (JPEG/PNG) on your host, e.g. `~/ros1_images`.
- An output folder for tagged images, e.g. `~/ros1_img_dst`.

---

## Build the Docker image

```bash
cd ~/ros1_docker_ws
# (Ensure your package source is in src/)
docker build -t sv01_uav_bs_transmission:latest .
```

This builds an image containing ROS Noetic, your package (with custom messages), OpenCV, Pillow, cv_bridge, and ExifTool.

---

## Run everything locally (single‐machine)

1. **Prepare host folders**

   ```bash
   mkdir -p ~/ros1_images ~/ros1_img_dst
   # Copy some .jpg/.jpeg/.png files into ~/ros1_images
   ```

2. **Start the ROS master in Docker**

   ```bash
   docker run -d --name ros_master \
     --network host \
     --entrypoint bash \
     sv01_uav_bs_transmission:latest \
     -lc "source /opt/ros/noetic/setup.bash && roscore"
   ```

3. **Launch the image publisher**

   ```bash
   docker run --rm --name ros_pub \
     --network host \
     -e ROS_MASTER_URI=http://127.0.0.1:11311 \
     -e ROS_IP=127.0.0.1 \
     -e NODE=image_publisher.py \
     -v ~/ros1_images:/root/catkin_ws/images:ro \
     sv01_uav_bs_transmission:latest
   ```

   The publisher will watch `/root/catkin_ws/images` for new files and publish up to **10 Hz**.

4. **Launch the image subscriber**

   ```bash
   docker run --rm --name ros_sub \
     --network host \
     -e ROS_MASTER_URI=http://127.0.0.1:11311 \
     -e ROS_IP=127.0.0.1 \
     -e NODE=image_subscriber.py \
     -v ~/ros1_img_dst:/root/catkin_ws/img_dst \
     sv01_uav_bs_transmission:latest
   ```

   The subscriber will save tagged JPEGs (and `.json` sidecars) into `~/ros1_img_dst`.

5. **Monitor logs**

   ```bash
   docker logs -f ros_pub
   docker logs -f ros_sub
   ```

   You should see:
   - **ros_pub**: `[INFO] Published <filename> with <N> metadata entries` (10 messages/sec)
   - **ros_sub**: `[INFO] Saved & tagged <filename>` as files appear.

6. **Verify output**

   ```bash
   ls ~/ros1_img_dst/*.jpg
   ```

   Open any of the saved images to confirm EXIF metadata is embedded.

---

7. **Frequency**
```bash
docker exec -it ros_master bash -lc "\
  source /opt/ros/noetic/setup.bash && \
  rostopic hz /image_meta"
 ```


## Run across two machines (drone + base station)

1. **On the base station (laptop)**
   ```bash
   # Start master in Docker
   docker run -d --name ros_master --network host --entrypoint bash \
     sv01_uav_bs_transmission:latest \
     -lc "source /opt/ros/noetic/setup.bash && roscore"
   ```

2. **On the drone**
   ```bash
   # Publish images from the drone
   docker run --rm --name ros_pub --network host \
     -e ROS_MASTER_URI=http://<BS_IP>:11311 \
     -e ROS_IP=<DRONE_IP> \
     -e NODE=image_publisher.py \
     -v /path/to/images:/root/catkin_ws/images:ro \
     sv01_uav_bs_transmission:latest
   ```

3. **Back on the base station**
   ```bash
   # Subscribe and save tagged images
   docker run --rm --name ros_sub --network host \
     -e ROS_MASTER_URI=http://<BS_IP>:11311 \
     -e ROS_IP=<BS_IP> \
     -e NODE=image_subscriber.py \
     -v /path/to/output:/root/catkin_ws/img_dst \
     sv01_uav_bs_transmission:latest
   ```

Replace `<BS_IP>` and `<DRONE_IP>` with the appropriate host IPs.

---

## Clean up

To stop and remove all containers:

```bash
docker rm -f ros_master ros_pub ros_sub
```

---



