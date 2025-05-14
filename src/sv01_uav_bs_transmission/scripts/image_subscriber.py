#!/usr/bin/env python3
import os
import json
import subprocess
import threading
import queue
import rospy
import numpy as np
import cv2
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp

# Subscriber node that receives ImageWithMetadataComp and embeds metadata back into images
# using ExifTool JSON interface.

topic_param = '~topic'
out_folder_param = '~output_folder'

# Background queue for disk writes
disk_write_queue = None


def compressed_to_cv2(img_msg):
    """
    Decode sensor_msgs/CompressedImage data to an OpenCV BGR image.
    """
    try:
        arr = np.frombuffer(img_msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("Decoded image is None")
        return img
    except Exception as e:
        rospy.logerr(f"Error decoding image: {e}")
        return None


def embed_exif(json_path, img_path):
    """
    Use ExifTool to write metadata from JSON into the given image file.
    """
    subprocess.run([
        'exiftool',
        f'-json={json_path}',
        '-overwrite_original',
        img_path
    ], check=True)


def save_with_metadata(cv_img, exif_dict, out_folder, filename):
    """
    Save OpenCV image to disk and embed metadata via ExifTool.
    """
    os.makedirs(out_folder, exist_ok=True)
    img_path = os.path.join(out_folder, filename)

    # Determine extension and encode image accordingly
    ext = os.path.splitext(filename)[1].lower() or '.jpg'
    success, buf = cv2.imencode(ext, cv_img)
    if success:
        with open(img_path, 'wb') as f:
            f.write(buf)
    else:
        cv2.imwrite(img_path, cv_img)

    # Write metadata JSON file for ExifTool
    json_path = img_path + '.json'
    with open(json_path, 'w') as jf:
        json.dump([exif_dict], jf, indent=4)

    # Embed metadata
    try:
        embed_exif(json_path, img_path)
        rospy.loginfo(f"Embedded metadata into {filename}")
    except Exception as e:
        rospy.logerr(f"ExifTool embedding failed for {filename}: {e}")


def disk_writer_worker(q, out_folder):
    """
    Worker thread that processes the disk_write_queue.
    """
    while not rospy.is_shutdown():
        try:
            cv_img, exif_dict, fname = q.get(timeout=1)
        except queue.Empty:
            continue
        try:
            save_with_metadata(cv_img, exif_dict, out_folder, fname)
        except Exception as e:
            rospy.logerr(f"Error saving {fname}: {e}")
        finally:
            q.task_done()


def image_callback(msg):
    """
    Callback invoked on arrival of an ImageWithMetadataComp message.
    Uses the 'FileName' metadata to ensure naming consistency.
    """
    cv_img = compressed_to_cv2(msg.image)
    if cv_img is None:
        return

    # Reconstruct metadata dictionary
    exif_dict = {k: v for k, v in zip(msg.metadata_keys, msg.metadata_values)}

    # Use the exact FileName published
    filename = exif_dict.get('FileName')
    if not filename:
        rospy.logerr("Received image without 'FileName' metadata; skipping.")
        return

    # Enqueue for background saving
    try:
        disk_write_queue.put((cv_img, exif_dict, filename), block=True, timeout=1)
        rospy.loginfo(f"Queued {filename} for saving and embedding")
    except queue.Full:
        rospy.logwarn(f"Disk write queue full, dropping image {filename}")


def main():
    rospy.init_node('image_subscriber', anonymous=True)
    topic = rospy.get_param(topic_param, 'image_meta')
    out_folder = rospy.get_param(out_folder_param, '/root/catkin_ws/img_dst')

    rospy.loginfo(f"Subscribing to '{topic}', saving to '{out_folder}'")

    global disk_write_queue
    # Buffer large images
    disk_write_queue = queue.Queue(maxsize=500)

    # Start background disk writer thread
    writer = threading.Thread(
        target=disk_writer_worker,
        args=(disk_write_queue, out_folder),
        daemon=True
    )
    writer.start()

    # Subscribe with large internal buffer to avoid drops
    rospy.Subscriber(
        topic,
        ImageWithMetadataComp,
        image_callback,
        queue_size=200,
        buff_size=50 * 1024 * 1024  # 50 MB
    )

    rospy.spin()

if __name__ == '__main__':
    main()
