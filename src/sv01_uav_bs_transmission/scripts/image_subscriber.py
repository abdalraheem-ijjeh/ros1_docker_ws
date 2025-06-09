#!/usr/bin/env python3
"""
ROS Noetic base‑station image subscriber – thumbnail‑embed hotfix
================================================================

Receives `ImageWithMetadataComp`, up‑scales the preview, **embeds every tag**
AND forces the thumbnail into the JPEG using ExifTool’s
`-ThumbnailImage<=file` syntax (more reliable than JSON import).

2025‑06‑09 **Fix D**
-------------------
* After writing the image we **decode `ThumbnailImageData`, save it to a temp
  file, and call**
  ```bash
  exiftool -overwrite_original -ThumbnailImage<=temp.jpg target.jpg
  ```
  which embeds the thumbnail even when the file had no EXIF block before.
* Other tags are still written via the JSON import.

Parameters stay the same (`~output_folder`, `~upscale_factor`, `~queue_size`).
"""

from __future__ import annotations

import os
import subprocess
import queue
import threading
import json
import base64
import tempfile
from typing import Dict, Any

import cv2
import numpy as np
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp

# ───────────────────────────── helpers ───────────────────────────────────────

def compressed_to_cv2(comp_msg) -> np.ndarray:
    buf = np.frombuffer(comp_msg.data, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def prepare_exif_for_import(exif_dict: Dict[str, Any], target_path: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"SourceFile": target_path}
    skip = {"Directory", "FileName", "ThumbnailImageData"}
    for k, v in exif_dict.items():
        if k in skip:
            continue
        out[k] = v if isinstance(v, (dict, list, int, float)) else str(v)
    return out


def embed_exif(json_path: str, img_path: str) -> None:
    res = subprocess.run(
        ["exiftool", f"-json={json_path}", "-overwrite_original", "-preserve", img_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if res.returncode != 0:
        raise subprocess.CalledProcessError(res.returncode, res.args, res.stdout, res.stderr)


def embed_thumbnail(thumb_b64: str, img_path: str) -> None:
    if not thumb_b64:
        return
    try:
        thumb_bytes = base64.b64decode(thumb_b64)
    except base64.binascii.Error:
        rospy.logwarn("Bad base‑64 thumbnail – skipping")
        return
    if not thumb_bytes:
        return
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
        tf.write(thumb_bytes)
        tmp_name = tf.name
    try:
        res = subprocess.run(
            ["exiftool", "-overwrite_original", f"-ThumbnailImage<={tmp_name}", img_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if res.returncode != 0:
            rospy.logwarn(f"Thumbnail embed failed: {res.stderr.decode().strip()}")
    finally:
        os.unlink(tmp_name)


def save_with_metadata(cv_img: np.ndarray, exif_dict: Dict[str, Any], out_dir: str, filename: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    img_path = os.path.join(out_dir, filename)

    # 1) save the (up‑scaled) JPEG
    ext = os.path.splitext(filename)[1].lower() or ".jpg"
    ok, buf = cv2.imencode(ext, cv_img)
    if not ok:
        raise IOError("cv2.imencode failed")
    with open(img_path, "wb") as fh:
        fh.write(buf)

    # 2) write all textual/numeric tags via JSON import
    exif_clean = prepare_exif_for_import(exif_dict, img_path)
    json_path = img_path + ".json"
    with open(json_path, "w", encoding="utf-8") as jf:
        json.dump([exif_clean], jf, indent=4, ensure_ascii=False)
    try:
        embed_exif(json_path, img_path)
    finally:
        os.remove(json_path)

    # 3) force‑embed the thumbnail
    embed_thumbnail(exif_dict.get("ThumbnailImageData", ""), img_path)

# ───────────────────── writer thread ─────────────────────────────────────────

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

# ─────────────────────────── ROS callback ────────────────────────────────────

def make_callback(output_root: str, upscale: float, q: queue.Queue):
    def image_cb(msg: ImageWithMetadataComp) -> None:
        small = compressed_to_cv2(msg.image)
        if small is None:
            rospy.logwarn("Decode failure – skipping frame")
            return
        h_s, w_s = small.shape[:2]
        big = cv2.resize(small, (max(1, int(w_s * upscale)), max(1, int(h_s * upscale))),
                         interpolation=cv2.INTER_CUBIC)
        exif_dict: Dict[str, Any] = {k: json.loads(v) for k, v in zip(msg.metadata_keys, msg.metadata_values)}
        filename = exif_dict.get("FileName") or f"{msg.image.header.stamp.to_nsec()}.{msg.image.format}"
        out_dir = os.path.join(output_root, msg.drone_id or "unknown", f"flight_{msg.flight_num or 0}")
        try:
            q.put((big, exif_dict, filename, out_dir), block=False)
        except queue.Full:
            rospy.logwarn("Writer queue full – dropping frame")
    return image_cb

# ─────────────────────────────── main ───────────────────────────────────────

def main() -> None:
    rospy.init_node("image_subscriber", anonymous=True)
    output_root = rospy.get_param("~output_folder", "/tmp/drone_images")
    upscale = max(0.1, float(rospy.get_param("~upscale_factor", 4.0)))
    qsize = int(rospy.get_param("~queue_size", 500))

    q = queue.Queue(maxsize=qsize)
    threading.Thread(target=disk_writer_worker, args=(q,), daemon=True).start()

    rospy.Subscriber("image_meta", ImageWithMetadataComp,
                     make_callback(output_root, upscale, q),
                     queue_size=200, buff_size=50*1024*1024)
    rospy.loginfo(f"📥 subscriber ready → {output_root} (upscale={upscale})")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
