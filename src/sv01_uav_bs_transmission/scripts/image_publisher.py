#!/usr/bin/env python3
"""
ROS Noetic UAV image‑publisher (robust thumbnail)
==============================================

Watches a folder for new images, extracts *all* EXIF/XMP tags, guarantees a
base‑64 **ThumbnailImageData** field, re‑encodes the picture at user‑defined
scale/quality and publishes an `ImageWithMetadataComp` message.

2025‑06‑09 **Thumbnail hard‑guarantee patch**
-------------------------------------------
* **Never sends an empty thumbnail** – if the file has no embedded thumbnail OR
  OpenCV cannot generate one, we create a 1×1 white JPEG placeholder so the
  subscriber can always embed something.
* Adds verbose logging: the line `Published … (thumbnail X kB …)` will now show
  `0 kB` **only if generation genuinely failed**.
* No change to message structure or parameters.
"""

import os
import subprocess
import json
import base64
import time
from typing import Dict, Any, Tuple

import cv2
import numpy as np
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp

# ─────────────────────────── helpers ────────────────────────────────────────

def ensure_thumbnail(image_path: str) -> bytes:
    """Return **non‑empty** JPEG bytes – embedded, generated, or 1×1 fallback."""
    # 1) try embedded thumbnail via ExifTool
    try:
        emb = subprocess.check_output(["exiftool", "-b", "-ThumbnailImage", image_path])
        if emb:
            return emb
    except subprocess.CalledProcessError:
        pass  # no thumbnail

    # 2) generate quick thumbnail (~160 px wide)
    np_arr = np.fromfile(image_path, dtype=np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img_bgr is not None:
        h, w = img_bgr.shape[:2]
        new_w = min(160, w)
        new_h = max(1, int(h * new_w / w))
        thumb = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", thumb, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        if ok and buf.size:
            return buf.tobytes()

    # 3) last‑resort 1×1 white JPEG
    one_px = np.full((1, 1, 3), 255, np.uint8)
    ok, buf = cv2.imencode(".jpg", one_px)
    return buf.tobytes() if ok else b"\xff\xd8\xff\xd9"  # minimal JPEG


def extract_metadata(image_path: str) -> Dict[str, Any]:
    """Return dict of tags + guaranteed `ThumbnailImageData` base64 string."""
    try:
        raw = subprocess.check_output(["exiftool", "-json", image_path])
        meta = json.loads(raw.decode("utf-8"))[0]
    except subprocess.CalledProcessError:
        meta = {}

    # Normalise path fields
    meta.pop("SourceFile", None)
    meta["Directory"] = "."

    # Insert thumbnail
    thumb_bytes = ensure_thumbnail(image_path)
    meta["ThumbnailImageData"] = base64.b64encode(thumb_bytes).decode("ascii")

    # Ensure JSON‑safe values
    safe_meta = {k: v if isinstance(v, (dict, list, int, float)) else str(v) for k, v in meta.items()}
    return safe_meta


def reencode_image(path: str, file_bytes: bytes, scale_factor: float, jpeg_quality: int) -> Tuple[bytes, str]:
    np_arr = np.frombuffer(file_bytes, dtype=np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img_bgr is None:
        return file_bytes, path.split(".")[-1].lower()
    h, w = img_bgr.shape[:2]
    new_w = max(1, int(w * scale_factor))
    new_h = max(1, int(h * scale_factor))
    resized = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    return (buf.tobytes(), "jpg") if ok else (file_bytes, path.split(".")[-1].lower())

# ─────────────────────────── main loop ──────────────────────────────────────

def image_publisher():
    rospy.init_node("image_publisher_uav", anonymous=True)

    drone_id = rospy.get_param("~drone_id", "drone1")
    flight_num = int(rospy.get_param("~flight_num", 1))
    image_dir = rospy.get_param("~image_folder", "/root/catkin_ws/images")

    scale_factor = float(rospy.get_param("~scale_factor", 0.25))
    if not 0.0 < scale_factor <= 1.0:
        rospy.logwarn(f"Invalid ~scale_factor {scale_factor}; using 0.25")
        scale_factor = 0.25

    jpeg_quality = int(rospy.get_param("~jpeg_quality", 70))
    if not 1 <= jpeg_quality <= 100:
        rospy.logwarn(f"Invalid ~jpeg_quality {jpeg_quality}; using 70")
        jpeg_quality = 70

    pub = rospy.Publisher("image_meta", ImageWithMetadataComp, queue_size=50)
    rate = rospy.Rate(1.0)
    already_sent = set()

    rospy.loginfo(
        f"image_publisher_uav ready – dir='{image_dir}' scale={scale_factor} quality={jpeg_quality}"
    )

    while not rospy.is_shutdown():
        try:
            files = sorted(f for f in os.listdir(image_dir) if f.lower().endswith((".jpg", ".jpeg", ".png")))
        except Exception as exc:
            rospy.logerr(f"Cannot list {image_dir}: {exc}")
            rate.sleep()
            continue

        for fn in files:
            if fn in already_sent:
                continue
            full = os.path.join(image_dir, fn)
            try:
                raw_bytes = open(full, "rb").read()
            except Exception as exc:
                rospy.logwarn(f"Cannot read {full}: {exc}")
                continue

            comp_bytes, fmt = reencode_image(full, raw_bytes, scale_factor, jpeg_quality)
            meta = extract_metadata(full)
            m_keys = sorted(meta)
            m_vals = [json.dumps(meta[k], ensure_ascii=False) for k in m_keys]

            msg = ImageWithMetadataComp()
            msg.image.header.stamp = rospy.Time.now()
            msg.image.format = fmt
            msg.image.data = comp_bytes
            msg.metadata_keys = m_keys
            msg.metadata_values = m_vals
            msg.drone_id = drone_id
            msg.flight_num = flight_num

            pub.publish(msg)
            thumb_kb = len(meta["ThumbnailImageData"]) * 3 // 4 // 1024
            rospy.loginfo(f"Published {fn} (thumbnail {thumb_kb} kB, {len(m_keys)} tags)")
            already_sent.add(fn)

        rate.sleep()


if __name__ == "__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
