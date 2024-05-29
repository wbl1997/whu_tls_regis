
import os
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python {} <time_offset_file>".format(sys.argv[0]))
        sys.exit(1)

    time_offset_file = sys.argv[1]
    if not os.path.exists(time_offset_file):
        print("Error: time offset file {} does not exist".format(time_offset_file))
        sys.exit(1)

    outputfile = time_offset_file.replace('.log', '_gist.txt')

    with open(time_offset_file, 'r') as f:
        lines = f.readlines()
    with open(outputfile, 'w') as f:
        f.write("#bagname lidar_frame_time_offset_to_zed_imu(sec) std(sec)\n")
        f.write("#the lidar frame timestamp + time offset = lidar frame timestamp per the zed imu clock.\n")
        for line in lines:
            if line.startswith('Aligning') and ".csv" in line:
                parts = line.split()
                path = parts[1]
                segments = path.split('/')
                run = segments[-2]
                date = segments[-3]
                f.write('{}/{} '.format(date, run))
            if '+ time offset' in line:
                parts = line.split()
                time_offset = parts[-5]
                std = parts[-2]
                f.write('{} {}\n'.format(time_offset, std))

