#!/bin/bash
# run this shell from whu_tls_regis folder.

datadir="/media/jhuai/BackupPlus/jhuai/data/homebrew/zongmu"
init_pose_dir="/media/jhuai/BackupPlus/jhuai/results/front_back_snapshots"
backduration=240

bags202312=(20231201/data2
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


bag20231105=(
20231105/data1
20231105/data2
20231105/data3
20231105/data4
20231105/data5
20231105/data6
20231105/data7)

bag20231105_aft=(20231105_aft/data1
20231105_aft/data2
20231105_aft/data3
20231105_aft/data4
20231105_aft/data5
20231105_aft/data6)

bag20231109=(20231109/data1
20231109/data2
20231109/data3
20231109/data4)


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
  python3 reverse_bag.py $bagfile $outbagfile /hesai/pandar /mti3dk/imu $backduration
done
}

reverse_bags "${bags202312[@]}"
