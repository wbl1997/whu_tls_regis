# whu_tls_regis
1. Register pairs of TLS lidar scans with point to plane ICP in open3d.
2. Also regularize the TLS poses with loop constraints with PGO in gtsam.
3. Use point to plane ICP in open3d to refine the start or the end pose of a data collection session.

# Dependency open3d and gtsam
```
## install miniconda and create a virtual env dataprep
## conda install open3d
## pip install gtsam

```

# Data description
The data were collected by Leica RTC360 by Xiaochen Wang and Jianzhu Huai on Oct 6 2023, 
postprocessed by cyclone software by Hanwen Qi.

*Project1*

61 TLS scans.

A loop around xinghu lake, the info faculty soccer field, starting from the basketball court.

*Project2*

32 TLS scans.

1-22 a loop around software institute bldg, xinghu bldg, 
and 27-32 the underground parkinglot of xinghu bldg, 
starting from the baseketball court.

# Initial lidar pose refinement
Given the initial lidar pose, and the corresponding TLS scan, refine the lidar pose wrt the TLS scan.
