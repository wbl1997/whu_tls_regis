
# note: conda activate ros_env
script=/home/pi/Documents/lidar/whu_tls_regis/refine_initial_lidar_pose.py
snapshotdir=/media/pi/BackupPlus/jhuai/results/front_back_snapshots

folders=(20240113 20240115 20240116 20240116_eve)

folders=(20231105 20231105_aft 20231109)

tls_pcd_dir=/media/pi/BackupPlus/jhuai/data/homebrew/whu_tls_1030

for folder in ${folders[@]}
do
    echo "processing $folder"
    python3 $script $snapshotdir/$folder $tls_pcd_dir $snapshotdir/$folder/coarse_tls_T_xt32.txt 0 2>&1 | tee $snapshotdir/$folder/refine_tls.log
done
