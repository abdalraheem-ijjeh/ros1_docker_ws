#!/usr/bin/env python3
"""
ROS Noetic image‑publisher for UAVs
==================================

Watches a local folder for new JPEG/PNG images, extracts their EXIF metadata
with ExifTool and publishes the picture plus metadata on the shared
`image_meta` topic using the custom message
`sv01_uav_bs_transmission/ImageWithMetadataComp`.

Changes compared to the previous revision
----------------------------------------
* **Fixed JSON round‑trip** – every metadata value is now serialised with
  ``json.dumps`` before publishing so that the subscriber can safely recover
  the original type with ``json.loads``.
* **Adds a proper ROS time‑stamp** in ``msg.image.header.stamp`` so the
  subscriber can still generate a filename even when no *FileName* tag is
  present.
* **Honours the advertised JPEG quality of 70 %** (instead of 95 %). This keeps
  the bandwidth reasonable while still giving visually good images.

Author: Abdalraheem Abdullah Yousef Ijjeh
"""

import os
import subprocess
import json
import rospy
import cv2
import numpy as np
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp


# ────────────────────────────── helpers ──────────────────────────────────────

def extract_metadata(image_path):
    """Return a dict with all EXIF fields from *image_path* (best‑effort)."""
    try:
        raw = subprocess.check_output(
            ["exiftool", "-json", image_path], stderr=subprocess.STDOUT
        )
        meta = json.loads(raw.decode("utf-8"))[0]

        # Normalise a couple of fields so the subscriber can rely on them
        meta["Directory"] = "."  # strip absolute paths
        meta.pop("SourceFile", None)

        # Convert *every* value to a JSON‑serialisable Python type (mostly str)
        safe_meta = {k: v if isinstance(v, (dict, list, int, float)) else str(v)
                     for k, v in meta.items()}
        return safe_meta

    except subprocess.CalledProcessError as exc:
        rospy.logerr(f"ExifTool error for {image_path}: {(exc.output or b'').decode()}")
        return {}
    except Exception as exc:
        rospy.logexception(f"Unexpected error extracting metadata: {exc}")
        return {}


def reencode_image(path, file_bytes):
    """Return *(bytes, format_str)* – JPEG @70 % or the original data on failure."""
    np_arr = np.frombuffer(file_bytes, dtype=np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img_bgr is None:
        rospy.logwarn(f"cv2.imdecode failed on {path}; sending raw bytes instead.")
        return file_bytes, path.split(".")[-1].lower()

    h, w = img_bgr.shape[:2]
    resized = cv2.resize(img_bgr, (w // 4, h // 4), interpolation=cv2.INTER_AREA)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
    ok, buf = cv2.imencode(".jpg", resized, encode_params)
    if not ok:
        rospy.logwarn(f"JPEG re‑encoding failed for {path}; sending raw bytes instead.")
        return file_bytes, path.split(".")[-1].lower()

    return buf.tobytes(), "jpg"


# ───────────────────────────── main loop ─────────────────────────────────────

def image_publisher():
    rospy.init_node("image_publisher_uav", anonymous=True)

    drone_id    = rospy.get_param("~drone_id", "drone1")
    flight_num  = int(rospy.get_param("~flight_num", 1))
    image_dir   = rospy.get_param("~image_folder", "/root/catkin_ws/images")

    pub  = rospy.Publisher("image_meta", ImageWithMetadataComp, queue_size=50)
    rate = rospy.Rate(1.0)  # Hz

    rospy.loginfo(
        f"image_publisher_uav started (drone_id='{drone_id}', flight_num={flight_num:02d}); "
        f"watching '{image_dir}'"
    )

    already_sent = set()

    while not rospy.is_shutdown():
        try:
            files = sorted(
                f for f in os.listdir(image_dir)
                if f.lower().endswith((".jpg", ".jpeg", ".png"))
            )
        except Exception as exc:
            rospy.logerr(f"Could not list folder {image_dir}: {exc}")
            rate.sleep()
            continue

        for fn in files:
            if fn in already_sent:
                continue
            full = os.path.join(image_dir, fn)

            try:
                with open(full, "rb") as fh:
                    raw_bytes = fh.read()
            except Exception as exc:
                rospy.logwarn(f"Cannot read {full}: {exc}")
                continue

            # image re‑encode (or fallback to raw)
            comp_bytes, fmt = reencode_image(full, raw_bytes)

            # metadata
            meta = extract_metadata(full)
            m_keys   = list(meta.keys())
            m_values = [json.dumps(meta[k], ensure_ascii=False) for k in m_keys]

            # ─────────────── build message ───────────────
            msg = ImageWithMetadataComp()
            msg.image.header.stamp = rospy.Time.now()
            msg.image.format = fmt
            msg.image.data   = comp_bytes
            msg.metadata_keys   = m_keys
            msg.metadata_values = m_values
            msg.drone_id   = drone_id
            msg.flight_num = flight_num

            pub.publish(msg)
            rospy.loginfo(
                f"Published {fn} ({len(m_keys)} metadata tags) as '.{fmt}'"
            )
            already_sent.add(fn)

            rate.sleep()

        rate.sleep()


if __name__ == "__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
