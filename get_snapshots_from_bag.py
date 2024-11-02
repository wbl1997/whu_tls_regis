"""
get a lidar scan and a zed2 left image at the beginning of a rosbag, and the end of a rosbag
"""

import os
import rosbag
import rospy

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import struct
import sensor_msgs.point_cloud2 as pc2


def write_hesai_pcd_file(msg, filename):
    """Write a PointCloud2 message to a PCD file.
    https://pointclouds.org/documentation/tutorials/pcd_file_format.html
    """
    hesai_fieldnames = ['x', 'y', 'z', 'intensity', 'timestamp', 'ring']
    hesai_fieldtypecodes = ('f', 'f', 'f', 'f', 'd', 'H') # https://docs.python.org/3/library/struct.html
    with open(filename, 'wb') as f:
        f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
        f.write(bytearray("VERSION 0.7\n", 'utf-8'))
        f.write(bytearray(f"FIELDS {' '.join(hesai_fieldnames)}\n", 'utf-8'))
        f.write(bytearray("SIZE 4 4 4 4 8 2\n", 'utf-8'))
        f.write(bytearray("TYPE F F F F F U\n", 'utf-8'))
        f.write(bytearray("COUNT 1 1 1 1 1 1\n", 'utf-8'))
        f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
        f.write(bytearray("HEIGHT 1\n", 'utf-8'))
        f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
        f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
        f.write(bytearray("DATA binary\n", 'utf-8'))
        for point in pc2.read_points(msg, field_names=hesai_fieldnames):
            for i, p in enumerate(point):
                f.write(struct.pack(hesai_fieldtypecodes[i], p))


def write_livox_ros_pcd_file(msg, filename):
    livox_ros_fields = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp']
    livox_ros_fieldcodes = ['f', 'f', 'f', 'f', 'B', 'B', 'd']
    
    with open(filename, 'wb') as f:
        # Write the PCD header
        f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
        f.write(bytearray("VERSION 0.7\n", 'utf-8'))
        f.write(bytearray(f"FIELDS {' '.join(livox_ros_fields)}\n", 'utf-8'))
        f.write(bytearray("SIZE 4 4 4 4 1 1 8\n", 'utf-8'))
        f.write(bytearray("TYPE F F F F U U F\n", 'utf-8'))
        f.write(bytearray("COUNT 1 1 1 1 1 1 1\n", 'utf-8'))
        f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
        f.write(bytearray("HEIGHT 1\n", 'utf-8'))
        f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
        f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
        f.write(bytearray("DATA binary\n", 'utf-8'))
        
        # Write the binary data for each point
        for point in pc2.read_points(msg, field_names=livox_ros_fields):
            for i, p in enumerate(point):
                f.write(struct.pack(livox_ros_fieldcodes[i], p))


def get_snapshots_from_bag(bagfile, outdir, image_topic, lidar_topic):
    print("get snapshots from bagfile:", bagfile, "image_topic:", image_topic, "lidar_topic:", lidar_topic)
    front_dir = os.path.join(outdir, 'front')
    back_dir = os.path.join(outdir, 'back')
    if not os.path.exists(front_dir):
        os.makedirs(front_dir)
    if not os.path.exists(back_dir):
        os.makedirs(back_dir)

    bag = rosbag.Bag(bagfile)
    topics = bag.get_type_and_topic_info()
    if image_topic not in topics.topics:
        print(f"image topic {image_topic} not in the bag")
        return

    image_lidar_pairs = []
    prev_time_and_type = []
    for topic, msg, t in bag.read_messages():
        if topic == image_topic:
            if len(prev_time_and_type) > 0:
                if prev_time_and_type[1] == 'lidar':
                    if abs(t - prev_time_and_type[0]) < rospy.Duration(0.2):
                        image_lidar_pairs.append((t, prev_time_and_type[0]))
                        prev_time_and_type = []
                    else:
                        prev_time_and_type = [t, 'image']
                else:
                    prev_time_and_type = [t, 'image']
            else:
                prev_time_and_type = [t, 'image']
        elif topic == lidar_topic:
            if len(prev_time_and_type) > 0:
                if prev_time_and_type[1] == 'image':
                    if abs(t - prev_time_and_type[0]) < rospy.Duration(0.2):
                        image_lidar_pairs.append((prev_time_and_type[0], t))
                        prev_time_and_type = []
                    else:
                        prev_time_and_type = [t, 'lidar']
                else:
                    prev_time_and_type = [t, 'lidar']
            else:
                prev_time_and_type = [t, 'lidar']

    front_times = image_lidar_pairs[5] # avoid the initial unstable data.
    back_times = image_lidar_pairs[-5]
    print('rosbag start time:', bag.get_start_time(), 'end time:', bag.get_end_time())
    print('front times: {} {}'.format(front_times[0].to_sec(), front_times[1].to_sec()))
    print('back times: {} {}'.format(back_times[0].to_sec(), back_times[1].to_sec()))
    bag.close()

    # save the image and the lidar scan at the front and the back
    bag = rosbag.Bag(bagfile)
    bridge = CvBridge()
    for topic, msg, t in bag.read_messages():
        if topic == image_topic:
            if abs(t - front_times[0]) < rospy.Duration(0.01):
                if topic.endswith('compressed'):
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg)
                image_path = os.path.join(front_dir, f'{t.secs}.{t.nsecs:09d}.jpg')
                cv2.imwrite(image_path, cv_image)
            elif abs(t - back_times[0]) < rospy.Duration(0.01):
                if topic.endswith('compressed'):
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg)
                image_path = os.path.join(back_dir, f'{t.secs}.{t.nsecs:09d}.jpg')
                cv2.imwrite(image_path, cv_image)
        elif topic == lidar_topic:
            if abs(t - front_times[1]) < rospy.Duration(0.01):
                lidar_path = os.path.join(front_dir, f'{t.secs}.{t.nsecs:09d}.pcd')
                if 'hesai' in topic:
                    write_hesai_pcd_file(msg, lidar_path)
                elif 'livox' in topic:
                    write_livox_ros_pcd_file(msg, lidar_path)
            elif abs(t - back_times[1]) < rospy.Duration(0.01):
                lidar_path = os.path.join(back_dir, f'{t.secs}.{t.nsecs:09d}.pcd')
                if 'hesai' in topic:
                    write_hesai_pcd_file(msg, lidar_path)
                elif 'livox' in topic:
                    write_livox_ros_pcd_file(msg, lidar_path)
    bag.close()


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print('Usage: python get_snapshots_from_bag.py <bagfile/folder> <outdir> '
             '[image_topic=/zed2i/zed_node/left_raw/image_raw_gray/compressed] [lidar_topic=/hesai/pandar]')
        sys.exit(1)

    bagfile = sys.argv[1]
    outdir = sys.argv[2]
    image_topic = '/zed2i/zed_node/left_raw/image_raw_gray/compressed'
    lidar_topic = '/hesai/pandar'
    if len(sys.argv) >= 4:
        image_topic = sys.argv[3]
    if len(sys.argv) >= 5:
        lidar_topic = sys.argv[4]

    if bagfile.endswith(".bag"):
        get_snapshots_from_bag(bagfile, outdir, image_topic, lidar_topic)
    else: # this is a folder, search for rosbags, and recursively get snapshots
        folder = bagfile
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.endswith("_aligned.bag"):
                    bagfullpath = os.path.join(root, file)
                    date = os.path.basename(root)
                    run = file.split('_')[0]
                    outfolder = os.path.join(outdir, date, run)
                    get_snapshots_from_bag(bagfullpath, outfolder, image_topic, lidar_topic)

