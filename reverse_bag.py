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
import scipy.spatial.transform
import sys
import struct

import numpy as np
import rosbag
import rospy
import time

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

    maxtimefile = os.path.join(outputdir, 'bag_maxtime.txt')
    with open(maxtimefile, 'w') as f:
        f.write(f'{t_max.secs}.{t_max.nsecs:09d}\n')
        f.write(f'{tmax2.secs}.{tmax2.nsecs:09d}\n')

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


def favor_mti3dk(bagfile):
    start = time.time()
    bag = rosbag.Bag(bagfile) # Opening a rosbag often takes very long.
    end = time.time()
    # print(f'Opening bag {bagfile} took {end - start:.2f} seconds')
    start = time.time()
    topics = bag.get_type_and_topic_info().topics
    end = time.time()
    # print(f'Getting topics took {end - start:.2f} seconds')

    bag.close()
    if '/mti3dk/imu' in topics:
        return '/mti3dk/imu'
    if '/x36d/imu' in topics:
        return '/x36d/imu'
    else:
        print('Error: No mti3dk or x36d IMU topic found in the bag')
        return ''

def parse_time(timestr):
    parts = timestr.split('.')
    secs = int(parts[0])
    l = len(parts[1])
    nsecs = int(parts[1]) * 10**(9 - l)
    return rospy.Time(secs, nsecs)


def save_initial_pose(initposefile, posestr, time):
    p = [float(x) for x in posestr[:3]]
    q = [float(x) for x in posestr[3:]]
    R = scipy.spatial.transform.Rotation.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    with open(initposefile, 'w') as f:
        # write the 4X4 transform matrix
        for i in range(3):
            for j in range(3):
                f.write(f'{T[i, j]:.9f} ')
            f.write(f'{T[i, 3]:.9f}\n')
        f.write(f'{time.secs}.{time.nsecs:09d}\n')

def reverse_bag2(input_bag, output_bag, lidar_topic, imu_topic, tmin, tmax, endposestr):
    """
    tmin, tmax: Time range to reverse.
    endposestr: The pose for the scan at tmax.
    """
    # save initial pose
    outputdir = os.path.dirname(output_bag)
    initposefile = os.path.join(outputdir, 'tls_T_xt32_0.txt')
    save_initial_pose(initposefile, endposestr, tmax)

    # get bag maxtime which is the mirror time
    bag = rosbag.Bag(input_bag)
    tmirror = rospy.Time(bag.get_end_time())
    bag.close()

    t2 = rospy.Duration(tmirror.secs, tmirror.nsecs) * 2
    tmirror2 = rospy.Time(t2.secs, t2.nsecs)

    inbag = rosbag.Bag(input_bag)
    lidardir = os.path.join(outputdir, 'lidar')
    if not os.path.exists(lidardir):
        os.makedirs(lidardir)
    imudatalist = []
    lidartimelist = []

    maxtimefile = os.path.join(outputdir, 'mirror_time.txt')
    with open(maxtimefile, 'w') as f:
        f.write(f'{tmirror.secs}.{tmirror.nsecs:09d}\n')
        f.write(f'{tmirror2.secs}.{tmirror2.nsecs:09d}\n')

    if use_pypcd:
        from pypcd4 import PointCloud

    for topic, msg, t in inbag.read_messages(topics=[lidar_topic, imu_topic]):
        if t < tmin:
            continue
        if t > tmax:
            break
        if topic == imu_topic:
            stamp = tmirror2 - msg.header.stamp
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
                new_pc, new_header = reverse_lidar_times_pypcd(msg, tmirror2)
                lidartimelist.append(new_header.stamp)
                lidarfile = os.path.join(lidardir, f'{new_header.stamp.secs}.{new_header.stamp.nsecs:09d}.pcd')
                new_pc.save(lidarfile)
            else:
                new_pc, new_header = reverse_lidar_times(msg, tmirror2)
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


def main_old():
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


def main():
    import argparse
    parser = argparse.ArgumentParser(description=('Reverse rosbags for lidar-inertial odometry.\n'
                                     'Example: python reverse_bag.py /media/pi/BackupPlus/jhuai/results/reftrajs\n'
                                     '/media/pi/My_Book/jhuai/data/zip /media/pi/BackupPlus/jhuai/results/front_back_snapshots\n'))
    parser.add_argument('reftraj_dir', type=str, help='Directory containing the reference trajectories for each bag')
    parser.add_argument('bagdir', type=str, help='Directory containing the rosbags')
    parser.add_argument('outputdir', type=str, help='Output directory for the front and back reversed rosbags')
    parser.add_argument('--buffer', type=float, default=8, help='Buffer time to add to the reversed segment')
    args = parser.parse_args()

    buffer = rospy.Duration(args.buffer)
    reftraj_files = []
    for root, dirs, files in os.walk(args.reftraj_dir):
        for file in files:
            if file.endswith('tls_T_xt32.txt'):
                reftraj_files.append(os.path.join(root, file))
    print(f'Found {len(reftraj_files)} reference trajectory files')

    bag2traj = {}
    for r, reffile in enumerate(reftraj_files):
        d = os.path.dirname(reffile)
        run = os.path.basename(d)
        date = os.path.basename(os.path.dirname(d))
        bagfile = os.path.join(args.bagdir, date, run + '.bag')
        if not os.path.isfile(bagfile):
            print(f'Warn: Bag file {bagfile} not found')
            continue
        bag2traj[bagfile] = reffile
    print(f'Found {len(bag2traj)} bag files with reference trajectories')

    # reverse the front and back segment of every rosbag
    bagfiles = list(bag2traj.keys())
    bagfiles = sorted(bagfiles)
    for i, bagfile in enumerate(bagfiles):
        print(f'Processing bag {i+1}/{len(bagfiles)}: {bagfile}')
        reftraj_file = bag2traj[bagfile]
        # load the reftraj file in TUM format
        times = []
        poses = []
        with open(reftraj_file, 'r') as f:
            for l in f:
                if l.startswith('#') or l.strip() == '':
                    continue
                parts = l.strip().split()
                time = parse_time(parts[0])
                times.append(time)
                poses.append(parts[1:])
        # check if there exist a large gap in times
        time_diffs = np.diff([t.to_sec() for t in times])
        max_diff_idx = np.argmax(time_diffs)
        max_diff = time_diffs[max_diff_idx]
        # find the second max
        time_diffs[max_diff_idx] = 0
        second_max_diff = np.max(time_diffs)

        front_endidx = max_diff_idx
        single = True
        if second_max_diff > 0.5:
            print(f'Error: Found two large gaps in the reference trajectory of {bagfile}')
            continue
        if max_diff < 0.5:
            print(f'Info: No large gap <={max_diff:.5f} in the reference trajectory of {bagfile}')
            front_endidx = len(times) - 1
            single = True
        else:
            print(f'Info: Found a large gap {max_diff:.5f} in the reference trajectory of {bagfile}')
            front_endidx = max_diff_idx
            print('time at maxdiff = {}.{:09d} and next time = {}.{:09d}'.format(
                  times[max_diff_idx].secs, times[max_diff_idx].nsecs, times[max_diff_idx+1].secs, times[max_diff_idx+1].nsecs))
            single = False

        d = os.path.dirname(reftraj_file)
        run = os.path.basename(d)
        date = os.path.basename(os.path.dirname(d))
        output_bag = os.path.join(args.outputdir, date, run, 'front', 'reversed.bag')
        lidar_topic = '/hesai/pandar'
        imu_topic = favor_mti3dk(bagfile)
        endpose = poses[front_endidx] # the pose for the first scan of the reversed bag.
        print('Reversing the front segment of {} from {}.{:09d} to {}.{:09d} with IMU topic {}'.format(
              bagfile, times[0].secs, times[0].nsecs, times[front_endidx].secs, times[front_endidx].nsecs, imu_topic))
        reverse_bag2(bagfile, output_bag, lidar_topic, imu_topic, times[0] - buffer, times[front_endidx], endpose)

        if not single:
            output_bag = os.path.join(args.outputdir, date, run, 'back', 'reversed.bag')
            endpose = poses[-1]
            # we add a buffer to the end time because we assume that the end of the trajectory is stationary.
            print('Reversing the back segment of {} from {}.{:09d} to {}.{:09d} with IMU topic {}'.format(
                  bagfile, times[front_endidx+1].secs, times[front_endidx+1].nsecs, times[-1].secs, times[-1].nsecs, imu_topic))
            reverse_bag2(bagfile, output_bag, lidar_topic, imu_topic, times[front_endidx+1] - buffer, times[-1] + buffer, endpose)


if __name__ == '__main__':
    main()
