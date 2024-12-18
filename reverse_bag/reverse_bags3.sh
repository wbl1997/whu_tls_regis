#!/bin/bash
# WARN: Deprecated
# run this shell from whu_tls_regis folder.
# conda activate ros_env

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/ebike"
init_pose_dir="/media/pi/BackupPlus/jhuai/results/front_back_snapshots"
backduration=240

bags202311=(
  20231105/data6
  20231105_aft/data4)

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

reverse_bags "${bags202311[@]}"
