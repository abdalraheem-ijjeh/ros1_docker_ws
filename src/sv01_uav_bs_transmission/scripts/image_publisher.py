#!/usr/bin/env python3
"""
ROS Noetic image-publisher for UAVs

Watches a local folder for new images, extracts EXIF metadata with ExifTool,
and publishes everything on the shared `image_meta` topic as
sv01_uav_bs_transmission/ImageWithMetadataComp.

Changes from the original version:
  • Adds `drone_id`  (string)  and `flight_num` (uint32) to each message.
  • Reads those values from private parameters ~drone_id and ~flight_num.
  • All other behaviour is unchanged.

Author: you
"""

import os
import subprocess
import json
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp


def extract_metadata(image_path):
    """
    Extract metadata from an image via ExifTool JSON output.
    Returns a dict of metadata fields.  Uncritical errors are logged and
    an empty dict is returned so the calling code can continue.
    """
    try:
        raw = subprocess.check_output(
            ['exiftool', '-json', image_path],
            stderr=subprocess.STDOUT
        )
        metadata_dict = json.loads(raw.decode('utf-8'))[0]

        # normalise and clean-up a few keys so they survive the round-trip
        metadata_dict['Directory'] = '.'
        metadata_dict.pop('SourceFile', None)
        safe_dict = {k: json.dumps(v, ensure_ascii=False) for k, v in metadata_dict.items()}
        return safe_dict


    except subprocess.CalledProcessError as e:

        rospy.logerr(f"ExifTool error for {image_path}: {(e.output or b'').decode()}")

        return {}

    except Exception as e:

        rospy.logexception(f"Unexpected error extracting metadata: {e}")

        return {}


def image_publisher():
    """
    Main loop: scan `folder_path` for *.jpg/png files that have not yet been
    published, build the ImageWithMetadataComp message and publish it.
    """
    rospy.init_node('image_publisher_uav', anonymous=True)

    # ──────────────────────────────────────────────────────────────────────────
    # NEW PARAMS:  who am I & which flight is this?
    drone_id   = rospy.get_param('~drone_id',  'drone1')
    flight_num = int(rospy.get_param('~flight_num', 1))
    image_folder = rospy.get_param('~image_folder',
                                   '/root/catkin_ws/images')
    # ──────────────────────────────────────────────────────────────────────────

    pub = rospy.Publisher('image_meta', ImageWithMetadataComp, queue_size=100)
    rate = rospy.Rate(1)                          # publish max one image / s

    published = set()
    rospy.loginfo(
        f"📤 image_publisher_uav started "
        f"(drone_id='{drone_id}', flight_num={flight_num}); "
        f"watching '{image_folder}'"
    )

    while not rospy.is_shutdown():
        try:
            files = sorted(
                f for f in os.listdir(image_folder)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))
            )
        except Exception as e:
            rospy.logerr(f"Could not list folder {image_folder}: {e}")
            rate.sleep()
            continue

        for fn in files:
            if fn in published:
                continue
            path = os.path.join(image_folder, fn)

            try:
                with open(path, 'rb') as f:
                    data = f.read()
            except Exception as e:
                rospy.logwarn(f"Cannot read file {path}: {e}")
                continue

            metadata_dict = extract_metadata(path)
            keys   = list(metadata_dict.keys())
            values = [str(metadata_dict[k]) for k in keys]

            # ───────────── construct and publish message ──────────────
            msg = ImageWithMetadataComp()
            msg.image.format = fn.split('.')[-1]      # e.g. 'jpg'
            msg.image.data   = data
            msg.metadata_keys    = keys
            msg.metadata_values  = values
            msg.drone_id     = drone_id               # ← NEW
            msg.flight_num   = flight_num             # ← NEW

            pub.publish(msg)
            rospy.loginfo(f"Published {fn} ({len(keys)} metadata fields)")
            published.add(fn)

            rate.sleep()

        rate.sleep()


if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
