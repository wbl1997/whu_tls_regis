#!/bin/bash
# run this shell from whu_tls_regis folder.

datadir="/media/jhuai/BackupPlus/jhuai/data/homebrew/zongmu"
datadir="/media/pi/My_Book/jhuai/data/zip"
init_pose_dir="/media/pi/BackupPlus/jhuai/results/front_back_snapshots"

backduration=100000

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

loc_check_bags=(
  20231105/data6
  20231109/data3
  20231109/data4
)

reverse_bags() {
bagnames=("${@}")
for bag in "${bagnames[@]}"; do
  echo "Reversing bag: $bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=$datadir/$bag.bag
  outbagfile=$init_pose_dir/$date/$run/back/"$run"_aligned.bag
  echo "bagfile: $bagfile"
  echo "outbagfile: $outbagfile"
  cmd="python3 reverse_bag.py $bagfile $outbagfile /hesai/pandar /mti3dk/imu $backduration"
  echo $cmd
  $cmd
done
}

# reverse_bags "${bags202312[@]}"
reverse_bags "${loc_check_bags[@]}"