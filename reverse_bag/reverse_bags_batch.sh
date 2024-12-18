#!/bin/bash
# run in ros_env

script=/home/$USER/Documents/lidar/whu_tls_regis/reverse_bag/reverse_bag.py

python3 $script /media/$USER/BackupPlus/jhuai/results/ref_trajs_all \
    /media/$USER/My_Book/jhuai/data/zip /media/$USER/BackupPlus/jhuai/results/front_back_snapshots 2>&1 | tee /media/$USER/BackupPlus/jhuai/results/front_back_snapshots/reverse.log

python3 $script /media/$USER/MyBookDuo/jhuai/results/ref_trajs_all \
    /media/$USER/MyBookDuo/jhuai/data/zip /media/$USER/MyBookDuo/jhuai/results/front_back_snapshots 2>&1 | tee /media/$USER/MyBookDuo/jhuai/results/front_back_snapshots/reverse.log

