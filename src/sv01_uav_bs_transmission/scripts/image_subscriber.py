#!/usr/bin/env python3
import os
import json
import subprocess
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp

def compressed_to_cv2(img_msg):
    arr = np.frombuffer(img_msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        rospy.logerr("Failed to decode image")
    return img

def embed_exif(json_path, img_path):
    subprocess.run([
        "exiftool",
        f"-json={json_path}",
        "-overwrite_original",
        img_path
    ], check=True)

def save_with_metadata(cv_img, exif_dict, out_folder, base_fname):
    os.makedirs(out_folder, exist_ok=True)
    img_path = os.path.join(out_folder, base_fname)
    pil = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
    pil.save(img_path)

    json_path = os.path.splitext(img_path)[0] + ".json"
    with open(json_path, 'w') as f:
        json.dump([exif_dict], f, indent=4)

    embed_exif(json_path, img_path)
    rospy.loginfo(f"Saved & tagged {img_path}")

def image_callback(msg):
    cv_img = compressed_to_cv2(msg.image)
    if cv_img is None:
        return
    exif = {k: v for k, v in zip(msg.metadata_keys, msg.metadata_values)}
    fname = exif.get('file_name', f"{rospy.Time.now().to_nsec()}.jpg")
    out_dir = rospy.get_param('~output_folder',
                              '/root/catkin_ws/img_dst')
    save_with_metadata(cv_img, exif, out_dir, fname)

def main():
    rospy.init_node('image_subscriber_uav', anonymous=True)
    topic = rospy.get_param('~topic', 'image_meta')
    rospy.Subscriber(topic, ImageWithMetadataComp, image_callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
