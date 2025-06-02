#!/usr/bin/env python3
"""
ROS Noetic base-station image subscriber
---------------------------------------

Subscribes to /image_meta (type sv01_uav_bs_transmission/ImageWithMetadataComp),
writes every frame under <output_root>/<drone_id>/flight_<n>/,
and embeds all metadata back into the saved image.

Parameters
~~~~~~~~~~
~output_folder   (str)  Root directory for all images   [default: /tmp/drone_images]
~queue_size      (int)  Max frames waiting for disk     [default: 500]
"""

# ── Python-3.8 typing retrofit ────────────────────────────────────────────────
from __future__ import annotations   # enables PEP-563 postponed evaluation

import os
import subprocess
import queue
import threading
import json
from typing import Dict, Any

import cv2
import numpy as np
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp


# ─────────────────────────────────── helpers ──────────────────────────────────
def compressed_to_cv2(comp_msg):
    """Convert sensor_msgs/CompressedImage → cv2 BGR ndarray."""
    buf = np.frombuffer(comp_msg.data, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def embed_exif(json_path: str, img_path: str) -> None:
    """Write all tags from *json_path* into *img_path* in-place."""
    subprocess.run(
        ['exiftool', f'-json={json_path}', '-overwrite_original', img_path],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def save_with_metadata(
    cv_img: np.ndarray,
    exif_dict: Dict[str, Any],
    out_dir: str,
    filename: str
) -> None:
    """Encode *cv_img* to disk and embed *exif_dict* via ExifTool."""
    os.makedirs(out_dir, exist_ok=True)
    img_path = os.path.join(out_dir, filename)

    # encode image
    ext = os.path.splitext(filename)[1].lower() or '.jpg'
    ok, buf = cv2.imencode(ext, cv_img)
    if not ok:
        raise IOError("cv2.imencode failed")

    with open(img_path, 'wb') as fh:
        fh.write(buf)

    # write one-object JSON for ExifTool
    json_path = img_path + '.json'
    with open(json_path, 'w', encoding='utf-8') as jf:
        json.dump([exif_dict], jf, indent=4, ensure_ascii=False)

    # embed and clean up
    embed_exif(json_path, img_path)
    os.remove(json_path)


# ───────────────────────────── disk-writer thread ─────────────────────────────
def disk_writer_worker(q: queue.Queue) -> None:
    while not rospy.is_shutdown():
        try:
            cv_img, exif_dict, fname, out_dir = q.get(timeout=1)
        except queue.Empty:
            continue
        try:
            save_with_metadata(cv_img, exif_dict, out_dir, fname)
            rospy.loginfo(f"✔ Saved {fname} → {out_dir}")
        except Exception as exc:
            rospy.logerr(f"✖ Failed to save {fname}: {exc}")


# ─────────────────────────────── ROS callback ────────────────────────────────
def make_callback(output_root: str, q: queue.Queue):
    def image_cb(msg: ImageWithMetadataComp) -> None:
        # 1) Decode the compressed bytes into a small (down-scaled) cv2 image
        cv_img_small = compressed_to_cv2(msg.image)
        if cv_img_small is None:
            rospy.logwarn("Decode failure – skipping frame")
            return

        # 2) Upscale by a factor of 4 (bilinear looked blurry; cubic is nicer)
        h_small, w_small = cv_img_small.shape[:2]
        cv_img_up = cv2.resize(
            cv_img_small,
            (w_small * 4, h_small * 4),
            interpolation=cv2.INTER_CUBIC,
        )

        # 3) Reconstruct metadata dict (undo json.dumps on publisher side)
        exif_dict: Dict[str, Any] = {
            k: json.loads(v) for k, v in zip(msg.metadata_keys, msg.metadata_values)
        }

        # 4) Determine output filename
        filename = exif_dict.get('FileName')
        if not filename:
            # fallback: ROS timestamp + extension
            filename = f"{msg.image.header.stamp.to_nsec()}.{msg.image.format}"

        # 5) Build output directory path
        drone   = msg.drone_id or 'unknown'
        flight  = f"flight_{msg.flight_num}" if msg.flight_num else "flight_0"
        out_dir = os.path.join(output_root, drone, flight)

        # 6) Enqueue for disk writing
        try:
            q.put((cv_img_up, exif_dict, filename, out_dir), block=False)
        except queue.Full:
            rospy.logwarn("Writer queue full – dropping frame")

    return image_cb


# ─────────────────────────────────── main ─────────────────────────────────────
def main() -> None:
    rospy.init_node('image_subscriber', anonymous=True)

    output_root = rospy.get_param('~output_folder', '/tmp/drone_images')
    qsize       = int(rospy.get_param('~queue_size', 500))

    disk_q = queue.Queue(maxsize=qsize)
    threading.Thread(target=disk_writer_worker, args=(disk_q,), daemon=True).start()

    rospy.Subscriber(
        'image_meta',
        ImageWithMetadataComp,
        make_callback(output_root, disk_q),
        queue_size=200,
        buff_size=50 * 1024 * 1024,
    )

    rospy.loginfo(f"📥 image_subscriber ready – writing to {output_root}")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
