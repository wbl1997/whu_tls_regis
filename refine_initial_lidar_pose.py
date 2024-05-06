import os
import sys
import copy

import numpy as np
import open3d as o3d

def get_tls_scanid(init_pose_file):
    # get the last nonempty line
    lastline = ""
    with open(init_pose_file, "r") as f:
        lines = f.readlines()
        for line in reversed(lines):
            if line.strip() != "":
                lastline = line
                break
    segments = lastline.split()
    projectid = segments[-2]
    scanid = segments[-1]
    return int(projectid), int(scanid)


def get_init_pose(init_pose_file):
    # get the first 12 doubles and reshape to a 3x4 matrix
    init_pose = []
    with open(init_pose_file, "r") as f:
        lines = f.readlines()
        valcount = 0
        lineid = 0
        while valcount < 12:
            line = lines[lineid]
            segments = line.split()
            for seg in segments:
                init_pose.append(float(seg))
                valcount += 1
            lineid += 1
        T3x4 = np.array(init_pose[:12]).reshape(3, 4)
        T = np.identity(4)
        T[0:3, :] = T3x4
        return T
    return np.identity(4)


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 refine_initial_lidar_pose.py <lidar_pcd_dir> <tls_pcd_dir=/whu_tls_1030> [default_init_pose.txt] [showresult=0]")
        sys.exit(1)
    lidar_pcd_dir = sys.argv[1]
    tls_pcd_dir = sys.argv[2]
    if len(sys.argv) > 3:
        default_init_pose_file = sys.argv[3]
    else:
        default_init_pose_file = ""
    if len(sys.argv) > 4:
        showresult = int(sys.argv[4])
    else:
        showresult = 0

    lidar_pcd_files = []
    for root, dirs, files in os.walk(lidar_pcd_dir):
        pcd_and_pose = ["", ""]
        for file in files:
            if file.endswith(".pcd"):
                if len(pcd_and_pose[0]) > 0:
                    print("Error: more than one pcd file in {}, extra {}".format(root, file))
                else:
                    pcd_and_pose[0] = os.path.join(root, file)
            if file.endswith("coarse_tls_T_xt32.txt"):
                if len(pcd_and_pose[1]) > 0:
                    print("Error: more than one pose file in {}, extra {}".format(root, file))
                else:
                    pcd_and_pose[1] = os.path.join(root, file)
        if len(pcd_and_pose[1]) == 0:
            pcd_and_pose[1] = default_init_pose_file
        if len(pcd_and_pose[0]) > 0 and len(pcd_and_pose[1]) > 0:
            lidar_pcd_files.append(pcd_and_pose)
    print("Found {} lidar pcd files".format(len(lidar_pcd_files)))
    for i, pcd_and_pose in enumerate(lidar_pcd_files):
        print("{}. pcd: {}, pose: {}".format(i, pcd_and_pose[0], pcd_and_pose[1]))
    for i, pcd_and_pose in enumerate(lidar_pcd_files):
        print("Processing pcd: {}, pose: {}".format(pcd_and_pose[0], pcd_and_pose[1]))
        projectid, scanid = get_tls_scanid(pcd_and_pose[1])
        print("Project ID: {}, Scan ID: {}".format(projectid, scanid))
        if projectid == 1:
            tls_pcd_file = os.path.join(tls_pcd_dir, "project{}/regis/{}.pcd".format(projectid, scanid))
        elif projectid == 2:
            tls_pcd_file = os.path.join(tls_pcd_dir, "project{}/regis/{}_uniform.pcd".format(projectid, scanid))
        else:
            print("Error: unknown project ID {}".format(projectid))
            continue
        lidar_pcd = o3d.io.read_point_cloud(pcd_and_pose[0])
        tls_pcd = o3d.io.read_point_cloud(tls_pcd_file)

        threshold = 0.05
        tls_T_lidar_init = get_init_pose(pcd_and_pose[1])
        print("Initial TLS to lidar transformation:")
        print(tls_T_lidar_init)
        tls_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        reg_p2l = o3d.pipelines.registration.registration_icp(
            lidar_pcd, tls_pcd, threshold, tls_T_lidar_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        tls_T_lidar = reg_p2l.transformation
        print("TLS to lidar transformation:")
        print(tls_T_lidar)
        print("{}".format(reg_p2l))
        if showresult:
            draw_registration_result(lidar_pcd, tls_pcd, tls_T_lidar)

        savedir = os.path.dirname(pcd_and_pose[0])
        savefile = os.path.join(savedir, "tls_T_xt32.txt")
        with open(savefile, 'w') as s:
            s.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(tls_T_lidar[0, 0], tls_T_lidar[0, 1], tls_T_lidar[0, 2], tls_T_lidar[0, 3]))
            s.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(tls_T_lidar[1, 0], tls_T_lidar[1, 1], tls_T_lidar[1, 2], tls_T_lidar[1, 3]))
            s.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(tls_T_lidar[2, 0], tls_T_lidar[2, 1], tls_T_lidar[2, 2], tls_T_lidar[2, 3]))
            s.write("{} {}\n".format(projectid, scanid))
            s.write("#{}\n".format(reg_p2l))
            print("TLS to lidar transformation saved to {}".format(savefile))
    print("All done")
