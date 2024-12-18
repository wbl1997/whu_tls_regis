#!/bin/bash
# WARN: Deprecated
# run this shell from whu_tls_regis folder.

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/zongmu"
init_pose_dir="/media/pi/BackupPlus/jhuai/results/front_back_snapshots"
backduration=240

bags202312=(
  20231201/data2
  20231201/data3
  20231208/data1
  20231208/data2
  20231208/data3
  20231208/data4
  20231208/data5
  20231213/data1
  20231213/data2
  20231213/data3
  20231213/data4
  20231213/data5)

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

reverse_bags "${bags202312[@]}"
