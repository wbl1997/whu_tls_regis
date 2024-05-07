"""
create a new rosbag suitable for reverse order lidar-inertial odometry.
Basic idea:
1. For an accelerometer reading, change the time from t to 2 * t_max - t
2. For a gyro reading, change the time from t to 2 * t_max - t, and make the values negative.
3. For a lidar point cloud, change the time of each point and the message to 2 * t_max - t.
We tried [pypcd4](https://github.com/MapIV/pypcd4), but it seems buggy.

"""
import copy
import os
import sys
import struct

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import std_msgs.msg

hesai_fieldnames = ['x', 'y', 'z', 'intensity', 'timestamp', 'ring']
hesai_fieldtypes = (np.float32, np.float32, np.float32, np.float32, np.float64, np.uint16)
hesai_fieldtypecodes = ('f', 'f', 'f', 'f', 'd', 'H')

hesai_field_with_attributes = [PointField('x', 0, PointField.FLOAT32, 1),
                         PointField('y', 4, PointField.FLOAT32, 1),
                         PointField('z', 8, PointField.FLOAT32, 1),
                         PointField('intensity', 16, PointField.FLOAT32, 1),
                         PointField('timestamp', 20, PointField.FLOAT64, 1),
                         PointField('ring', 28, PointField.UINT16, 1)]

use_pypcd = False
check_point = True

def modify_msg_times(msg, tmax2):
    """
    reverse time of each point and message to 2 * t_max - t
    """
    points = []
    for i, p in enumerate(pc2.read_points(msg, field_names = hesai_fieldnames, skip_nans=True)):
        time = rospy.Time(p[4])
        newtime = tmax2 - time
        point = [p[0], p[1], p[2], p[3], newtime.to_sec(), p[5]]
        points.append(point)
    header = copy.deepcopy(msg.header)
    header.stamp = tmax2 - msg.header.stamp
    return points, header


def reverse_lidar_times_pypcd(msg, tmax2):
    from pypcd4 import PointCloud
    points, header = modify_msg_times(msg, tmax2)
    # sanity checks
    global check_point
    if check_point:
        pc_cmp = PointCloud.from_msg(msg)
        pc_array = pc_cmp.numpy()
        pts_list = []
        for p in pc2.read_points(msg, field_names = hesai_fieldnames, skip_nans=False):
            pts_list.append(p)
        print('pc2 points {}, pypcd4 shape {}'.format(len(pts_list), pc_array.shape))
        print("First point, pc2: {},\npypcd4: {}".format(pts_list[0], np.transpose(pc_array[0, :])))
        print("Last point, pc2: {},\npypcd4: {}".format(pts_list[-1], pc_array[-1, :]))
        print("Oddly, pypcd4 reads many spurious fields as placeholder. Why?")
        check_point = False

    # save the points as a ros PointCloud2 message
    array = np.array(points)
    pc = PointCloud.from_points(array, hesai_fieldnames, hesai_fieldtypes)
    return pc, header

def reverse_lidar_times(msg, tmax2):
    points, header = modify_msg_times(msg, tmax2)
    pc2_msg = pc2.create_cloud(header, hesai_field_with_attributes, points)
    pc2_msg.is_dense = True
    return pc2_msg, header


def write_pcd_file(msg, fieldnames, filename):
    """Write a PointCloud2 message to a PCD file.
    https://pointclouds.org/documentation/tutorials/pcd_file_format.html
    """
    with open(filename, 'wb') as f:
        f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
        f.write(bytearray("VERSION 0.7\n", 'utf-8'))
        f.write(bytearray(f"FIELDS {' '.join(fieldnames)}\n", 'utf-8'))
        f.write(bytearray("SIZE 4 4 4 4 8 2\n", 'utf-8'))
        f.write(bytearray("TYPE F F F F F U\n", 'utf-8'))
        f.write(bytearray("COUNT 1 1 1 1 1 1\n", 'utf-8'))
        f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
        f.write(bytearray("HEIGHT 1\n", 'utf-8'))
        f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
        f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
        f.write(bytearray("DATA binary\n", 'utf-8'))
        for point in pc2.read_points(msg, field_names=fieldnames):
            for i, p in enumerate(point):
                f.write(struct.pack(hesai_fieldtypecodes[i], p))


def read_pcd_file(filename, fieldtypecodes):
    """Read a PCD file and return a list of points with their attributes."""
    with open(filename, 'rb') as file:
        fields = []
        points = []
        while True:
            line = file.readline().decode('utf-8')
            if 'FIELDS' in line:
                fields = line.split()[1:]
            elif 'SIZE' in line:
                sizes = [int(s) for s in line.split()[1:]]
            elif 'DATA' in line:
                data_type = line.split()[1]
                break
        # print(f'fields from pcd: {fields}')
        # print(f'sizes from pcd: {sizes}')
        while True:
            point_data = {}
            for i, field in enumerate(fields):
                value_bytes = file.read(sizes[i])
                if not value_bytes:
                    break
                value = struct.unpack(fieldtypecodes[i], value_bytes)[0]
                point_data[field] = value
            if not point_data:
                break
            points.append([point_data[field] for field in fields])
        return points, fields


def reverse_bag(input_bag, output_bag, lidar_topic, imu_topic, duration):
    """
    duration: Duration from the end.
    """
    bag = rosbag.Bag(input_bag)
    for topic, msg, t in bag.read_messages(topics=[lidar_topic, imu_topic]):
        t_min = t
        break
    t_max = rospy.Time(bag.get_end_time())
    bag.close()

    t2 = rospy.Duration(t_max.secs, t_max.nsecs) * 2
    tmax2 = rospy.Time(t2.secs, t2.nsecs)

    t_min = max(t_min, t_max - duration)
    print("t_min = {}.{:09d}, duration = {:.2f}, t_max = {}.{:09d}".format(
          t_min.secs, t_min.nsecs, duration.to_sec(), t_max.secs, t_max.nsecs))

    inbag = rosbag.Bag(input_bag)
    outputdir = os.path.dirname(output_bag)
    lidardir = os.path.join(outputdir, 'lidar')
    if not os.path.exists(lidardir):
        os.makedirs(lidardir)
    imudatalist = []
    lidartimelist = []

    if use_pypcd:
        from pypcd4 import PointCloud

    for topic, msg, t in inbag.read_messages(topics=[lidar_topic, imu_topic]):
        if t < t_min:
            continue
        if topic == imu_topic:
            stamp = tmax2 - msg.header.stamp
            imudata = Imu()
            imudata.header.frame_id = msg.header.frame_id
            imudata.header.seq = msg.header.seq
            imudata.header.stamp = stamp
            imudata.linear_acceleration.x = msg.linear_acceleration.x
            imudata.linear_acceleration.y = msg.linear_acceleration.y
            imudata.linear_acceleration.z = msg.linear_acceleration.z
            imudata.angular_velocity.x = -msg.angular_velocity.x
            imudata.angular_velocity.y = -msg.angular_velocity.y
            imudata.angular_velocity.z = -msg.angular_velocity.z
            imudatalist.append(imudata)
        elif topic == lidar_topic:
            if use_pypcd:
                new_pc, new_header = reverse_lidar_times_pypcd(msg, tmax2)
                lidartimelist.append(new_header.stamp)
                lidarfile = os.path.join(lidardir, f'{new_header.stamp.secs}.{new_header.stamp.nsecs:09d}.pcd')
                new_pc.save(lidarfile)
            else:
                new_pc, new_header = reverse_lidar_times(msg, tmax2)
                lidartimelist.append(new_header.stamp)
                lidarfile = os.path.join(lidardir, f'{new_header.stamp.secs}.{new_header.stamp.nsecs:09d}.pcd')
                write_pcd_file(new_pc, hesai_fieldnames, lidarfile)
    inbag.close()

    print(f'Writing to {output_bag}, #imu messages: {len(imudatalist)}, #lidar messages: {len(lidartimelist)}')
    outbag = rosbag.Bag(output_bag, 'w')
    for i in range(len(imudatalist) - 1, -1, -1):
        msg = imudatalist[i]
        outbag.write(imu_topic, msg, msg.header.stamp)
    j = 0

    for i in range(len(lidartimelist) - 1, -1, -1):
        stamp = lidartimelist[i]
        lidarfile = os.path.join(lidardir, f'{stamp.secs}.{stamp.nsecs:09d}.pcd')
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = "PandarXT-32"
        header.seq = j

        if use_pypcd:
            pc: PointCloud = PointCloud.from_path(lidarfile)
            pc2_msg = pc.to_msg(header)
        else:
            points, _ = read_pcd_file(lidarfile, hesai_fieldtypecodes)
            pc2_msg = pc2.create_cloud(header, hesai_field_with_attributes, points)
            pc2_msg.is_dense = True

        outbag.write(lidar_topic, pc2_msg, stamp)
        j += 1
    outbag.close()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: python reverse_bag.py <input_bag> <output_bag> [/hesai/pandar] [/mti3dk/imu] [back_duration=10sec]')
        sys.exit(1)
    
    input_bag, output_bag = sys.argv[1], sys.argv[2]
    if len(sys.argv) > 3:
        lidar_topic = sys.argv[3]
    else:
        lidar_topic = '/hesai/pandar'
    if len(sys.argv) > 4:
        imu_topic = sys.argv[4]
    else:
        imu_topic = '/mti3dk/imu'
    if len(sys.argv) > 5:
        duration = rospy.Duration(float(sys.argv[5]))
    else:
        duration = rospy.Duration(999999999)
    use_pypcd = False
    reverse_bag(input_bag, output_bag, lidar_topic, imu_topic, duration)
