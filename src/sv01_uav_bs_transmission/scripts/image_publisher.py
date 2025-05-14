#!/usr/bin/env python3
import os
import rospy
from sv01_uav_bs_transmission.msg import ImageWithMetadataComp
from PIL import Image
from PIL.ExifTags import TAGS

def extract_metadata(image_path):
    metadata = {}
    try:
        with Image.open(image_path) as img:
            exif = img._getexif() or {}
            for tag_id, value in exif.items():
                tag_name = TAGS.get(tag_id) or str(tag_id)
                metadata[str(tag_name)] = value
    except Exception as e:
        rospy.logwarn(f"extract_metadata: {e}")
    try:
        stat = os.stat(image_path)
        metadata['file_name']       = os.path.basename(image_path)
        metadata['file_size_bytes'] = stat.st_size
        metadata['last_modified']   = stat.st_mtime
    except Exception as e:
        rospy.logwarn(f"file stat error: {e}")
    keys   = list(metadata.keys())
    values = [str(metadata[k]) for k in keys]
    return keys, values

def image_publisher(folder_path):
    rospy.init_node('image_publisher_uav', anonymous=True)
    pub = rospy.Publisher('image_meta', ImageWithMetadataComp, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    published = set()  # track filenames already sent

    rospy.loginfo(f"Watching folder: {folder_path}")
    while not rospy.is_shutdown():
        try:
            files = sorted(f for f in os.listdir(folder_path)
                           if f.lower().endswith(('.jpg','.jpeg','.png')))
        except Exception as e:
            rospy.logerr(f"Could not list folder: {e}")
            rate.sleep()
            continue

        for fn in files:
            if fn in published:
                continue

            path = os.path.join(folder_path, fn)
            try:
                with open(path, 'rb') as f:
                    img_bytes = f.read()
            except Exception as e:
                rospy.logwarn(f"Could not read {fn}: {e}")
                continue

            keys, vals = extract_metadata(path)

            msg = ImageWithMetadataComp()
            msg.image.format      = os.path.splitext(fn)[1].lstrip('.')
            msg.image.data        = img_bytes
            msg.metadata_keys     = keys
            msg.metadata_values   = vals

            pub.publish(msg)
            rospy.loginfo(f"Published new image {fn} ({len(keys)} metadata entries)")
            published.add(fn)

            rate.sleep()
        rate.sleep()

if __name__ == '__main__':
    folder = rospy.get_param('~image_folder', '/root/catkin_ws/images')
    try:
        image_publisher(folder)
    except rospy.ROSInterruptException:
        pass
