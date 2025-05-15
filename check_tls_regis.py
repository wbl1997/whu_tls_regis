
import laspy
import open3d as o3d

from whu_tls_regis import read_las_topcd, VOXEL_SIZE
import os
import numpy as np


def data_down_sample(lasfile, outdir):
    if (os.path.exists(lasfile)==False):
        print(lasfile+"NOT FIND!!!")
        return

    A_pcd_raw = read_las_topcd(lasfile)
    # 0~1 random
    r = np.random.rand(1)
    g = np.random.rand(1)
    b = np.random.rand(1)
    A_pcd_raw.paint_uniform_color([r, g, b])

    o3d.io.write_point_cloud(os.path.join(outdir, 'raw.pcd'), A_pcd_raw)
    B_pcd_raw = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    o3d.io.write_point_cloud(os.path.join(outdir, "ds.pcd"), B_pcd_raw)


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <original_las_file> <output_dir>")
        print(f"Example: {sys.argv[0]} "
              "/media/jhuai/BackupPlus/jhuai/data/homebrew/whu_tls/project1/27.las "
              "/home/jhuai/Desktop/temp")
        sys.exit(1)

    lasfile = sys.argv[1]
    outdir  = sys.argv[2]
    data_down_sample(lasfile, outdir)

