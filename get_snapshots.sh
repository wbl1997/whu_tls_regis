# note conda activate ros_env
folders=(/media/pi/My_Book/jhuai/data/zongmu/20240113
/media/pi/My_Book/jhuai/data/zongmu/20240115
/media/pi/My_Book/jhuai/data/zongmu/20240116
/media/pi/My_Book/jhuai/data/zongmu/20240116_eve)

script=/home/pi/Documents/lidar/whu_tls_regis/get_snapshots_from_bag.py
for folder in ${folders[@]}
do
    echo "Processing $folder"
    python3 $script $folder /media/pi/BackupPlus/jhuai/results/front_back_snapshots
done
