#!/bin/bash
# run this shell from whu_tls_regis folder.

datadir="/media/pi/My_Book/jhuai/data/zongmu"
init_pose_dir="/media/pi/BackupPlus/jhuai/results/front_back_snapshots"
backduration=240

bags202401=(
  20240113/data1
  20240113/data2
  20240113/data3
  20240113/data4
  20240113/data5
  20240115/data1
  20240115/data2
  20240115/data3
  20240115/data4
  20240116/data2
  20240116/data3
  20240116/data4
  20240116/data5
  20240116_eve/data1
  20240116_eve/data2
  20240116_eve/data3
  20240116_eve/data4
  20240116_eve/data5)

script=/home/pi/Documents/lidar/whu_tls_regis/reverse_bag.py

reverse_bags() {
bagnames=("${@}")
for bag in "${bagnames[@]}"; do
  echo "Reversing bag: "$bag"_aligned.bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=$datadir/"$bag"_aligned.bag
  outbagfile=$init_pose_dir/$date/$run/back/"$run"_aligned.bag
  echo "bagfile: $bagfile"
  echo "outbagfile: $outbagfile"
  python3 $script $bagfile $outbagfile /hesai/pandar /mti3dk/imu $backduration
done
}

reverse_bags "${bags202401[@]}"
