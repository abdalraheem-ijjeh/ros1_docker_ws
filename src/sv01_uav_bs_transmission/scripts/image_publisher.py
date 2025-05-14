#!/usr/bin/env python3
import os
import subprocess
import json
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp


def extract_metadata(image_path):
    """
    Extract metadata from an image via exiftool JSON output.
    Returns a dict of metadata fields.
    """
    try:
        raw = subprocess.check_output(
            ['exiftool', '-json', image_path],
            stderr=subprocess.STDOUT
        )
        metadata_dict = json.loads(raw.decode('utf-8'))[0]

        # normalize and cleanup
        metadata_dict['Directory'] = '.'
        metadata_dict.pop('SourceFile', None)
        return metadata_dict

    except subprocess.CalledProcessError as e:
        err = (e.output or b""
              ).decode('utf-8', errors='ignore')
        rospy.logerr(f"ExifTool error for {image_path}: {err}")
        return {}
    except Exception as e:
        rospy.logexception(f"Unexpected error extracting metadata for {image_path}: {e}")
        return {}


def image_publisher(folder_path):
    rospy.init_node('image_publisher_uav', anonymous=True)
    pub = rospy.Publisher('image_meta', ImageWithMetadataComp, queue_size=100)
    rate = rospy.Rate(1)

    published = set()
    rospy.loginfo(f"Watching folder for images to publish: {folder_path}")

    while not rospy.is_shutdown():
        try:
            files = sorted(
                f for f in os.listdir(folder_path)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))
            )
        except Exception as e:
            rospy.logerr(f"Could not list folder {folder_path}: {e}")
            rate.sleep()
            continue

        for fn in files:
            if fn in published:
                continue
            path = os.path.join(folder_path, fn)

            try:
                with open(path, 'rb') as f:
                    data = f.read()
            except Exception as e:
                rospy.logwarn(f"Cannot read file {path}: {e}")
                continue

            metadata_dict = extract_metadata(path)
            keys = list(metadata_dict.keys())
            values = [str(metadata_dict[k]) for k in keys]

            msg = ImageWithMetadataComp()
            msg.image.format = fn.split('.')[-1]  # e.g. jpg
            msg.image.data = data
            msg.metadata_keys = keys
            msg.metadata_values = values

            pub.publish(msg)
            rospy.loginfo(f"Published {fn} with {len(keys)} metadata entries")
            published.add(fn)

            rate.sleep()

        rate.sleep()


if __name__ == '__main__':
    image_folder = rospy.get_param('~image_folder', '/root/catkin_ws/images')
    try:
        image_publisher(image_folder)
    except rospy.ROSInterruptException:
        pass
