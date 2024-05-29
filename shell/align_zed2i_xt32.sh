
# align zed2i imu data and kissicp lidar odometry for those seqs without mti3dk

zed2imudir=/media/jhuai/MyBookDuo/jhuai/results/zed2imu
kissicpdir=/media/jhuai/MyBookDuo/jhuai/results/kissicp

align_bag() {
  bagname=$1
  echo "Aligning zed and hesai in bag: $bagname"
  date=${bagname%%/*} # the first part of $bag
  run=${bagname#*/} # the second part of $bag
  posefile=$kissicpdir/$bagname/"$run"_aligned_poses_tum.txt
  imufile=$zed2imudir/$bagname/zed2i_imu.csv
  outputpath=$zed2imudir/$bagname
  set -x
  /usr/local/MATLAB/R2022a/bin/matlab -nodesktop -nosplash -r \
  "cd('/media/jhuai/Seagate2TB/jhuai/tools/viclam-scripts'); \
    try \
      estimateRotationAndTimeOffset('$posefile', '$imufile', '$outputpath'); \
    catch e; \
      msgText = getReport(e); \
      disp(['Error occurs: ' msgText]); \
    end; \
    quit";
}

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/handheld"
bagnames=(
"20230920/data1"
"20230920/data2"
"20230920/data3"
"20230921/data2"
"20230921/data3"
"20230921/data4"
"20230921/data5")

outputlog=$datadir/align_zed2i_xt32.log
rm -f $outputlog
count=0
for bagname in ${bagnames[@]}; do
    align_bag $bagname 2>&1 | tee -a $outputlog
    count=$((count+1))
    # if [ $count -eq 1 ]; then
    #     break
    # fi
done

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/ebike"
bagnames=("20231007/data1"
"20231007/data2"
"20231007/data3"
"20231007/data4"
"20231007/data5"
"20231019/data1"
"20231019/data2"
"20231025/data1"
"20231025/data2")

outputlog=$datadir/align_zed2i_xt32.log
rm -f $outputlog
count=0
for bagname in ${bagnames[@]}; do
    align_bag $bagname 2>&1 | tee -a $outputlog
    count=$((count+1))
    # if [ $count -eq 1 ]; then
    #     break
    # fi
done
