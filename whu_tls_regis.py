#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
whu_tls_rigis.py
"""

__author__  = "wbl"
__version__ = "1.00"
__date__    = "26 Oct 2023"

import copy
import os
import sys

import time
import numpy as np
import open3d as o3d
import laspy
import gtsam
import matplotlib.pyplot as plt
from scipy.linalg import svd

import icp_info

VOXEL_SIZE = 0.05
VISUALIZE = True
SAVE_INFO = False

def read_las_topcd(las_file, step_len=10):
    # 打开LAS文件
    las_reader = laspy.read(las_file)
    # 获取点云数据
    point_cloud = las_reader.points
    # 获取LAS文件的缩放因子和偏移值
    x_scale = las_reader.header.scale[0]
    y_scale = las_reader.header.scale[1]
    z_scale = las_reader.header.scale[2]

    x_offset = las_reader.header.offset[0]
    y_offset = las_reader.header.offset[1]
    z_offset = las_reader.header.offset[2]
    # 获取点的坐标，例如 X、Y、Z 坐标，并间隔10个点取一个点
    x = point_cloud['X'][::step_len]*x_scale + x_offset
    y = point_cloud['Y'][::step_len]*y_scale + y_offset
    z = point_cloud['Z'][::step_len]*z_scale + z_offset
    print("x: ", x.min(), x.max(), x.shape)
    # 将xyz转换为pcd格式
    ropcd = o3d.geometry.PointCloud()
    ropcd.points = o3d.utility.Vector3dVector(np.array([x, y, z]).T)
    return ropcd

def data_down_sample(path, filename_list):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
    for i in np.arange(0, len(filename_list)):
        fileCur = str(filename_list[i])
        print("fileCur: ", fileCur)
        # read las to pcd
        A_pcd_raw = read_las_topcd(path+fileCur+".las")
        # 0~1 random
        r = np.random.rand(1)
        g = np.random.rand(1)
        b = np.random.rand(1)
        A_pcd_raw.paint_uniform_color([r, g, b])

        A_pcd_raw = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
        o3d.io.write_point_cloud(path+"result/downsample/"+fileCur+".pcd", A_pcd_raw)

def whutls_regis_from_relative_trans(path, filename_list):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
    T_absolute = np.eye(4)
    for i in np.arange(0, len(filename_list)-1):
        fileCur = str(filename_list[i])
        fileNext = str(filename_list[i+1])
        if i==0:
            B_pcd_raw = o3d.io.read_point_cloud(path+"result/downsample/"+fileCur+".pcd")
            o3d.io.write_point_cloud(path+"result/regis/"+fileCur+".pcd", B_pcd_raw)
        # load relative trans from txt
        T_relative_save_path = path+"result/trans/"+fileCur+"_"+fileNext+".txt"
        T_relative = np.loadtxt(T_relative_save_path)
        T_relative.reshape(3, 4)
        T_relative = np.hstack((T_relative, np.array([0, 0, 0, 1]))).reshape(4, 4)
        T_absolute = np.dot(T_absolute, T_relative)
        # transform fileNext to fileCur frame
        A_pcd_raw = o3d.io.read_point_cloud(path+"result/downsample/"+fileNext+".pcd")
        A_pcd_raw.transform(T_absolute)
        o3d.io.write_point_cloud(path+"result/regis/"+fileNext+".pcd", A_pcd_raw)

def whutls_regis_from_absolute_trans(path, filename_list):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
    T_absolute = np.eye(4)
    for i in np.arange(0, len(filename_list)):
        # load relative trans from txt
        T_absolute_path = path+"result/trans/absolute_pgo/"+str(filename_list[i])+".txt"
        T_absolute = np.loadtxt(T_absolute_path)
        T_absolute.reshape(3, 4)
        T_absolute = np.hstack((T_absolute, np.array([0, 0, 0, 1]))).reshape(4, 4)
        # transform fileNext to fileCur frame
        A_pcd_raw = o3d.io.read_point_cloud(path+"result/downsample/"+str(filename_list[i])+".pcd")
        A_pcd_raw.transform(T_absolute)
        o3d.io.write_point_cloud(path+"result/regis/"+str(filename_list[i])+".pcd", A_pcd_raw)

def whutls_one2one_match(path, filename_list, SeqCur, SeqNext):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
   
    fileCur = str(SeqCur)
    fileNext = str(SeqNext)
    print("fileCur: ", fileCur, ", fileNext: ", fileNext)

    # read pcd
    B_pcd_raw = o3d.io.read_point_cloud(path+"result/downsample/"+fileCur+".pcd")
    A_pcd_raw = o3d.io.read_point_cloud(path+"result/downsample/"+fileNext+".pcd")

    # voxel downsample
    A_pcd = copy.deepcopy(A_pcd_raw)
    B_pcd = copy.deepcopy(B_pcd_raw)
    print("A_pcd: ", np.asarray(A_pcd.points).shape)
    print("B_pcd: ", np.asarray(B_pcd.points).shape)

    print("refine use icp!", fileCur, fileNext)
    result_T = np.array( [[ 1,   -0,      0,   -0.0],
                          [ 0.0,  1,      0,    0.0],
                          [ 0,    0,      1,    0.0],
                          [ 0.0,  0.0,  0.0,    1.0]]) 
    T_init_path = path+"result/init_trans/"+fileCur+"_"+fileNext+".txt"
    if (os.path.exists(T_init_path)==True):
        result_T = np.loadtxt(T_init_path)
        result_T.reshape(3, 4)
        result_T = np.hstack((result_T, np.array([0, 0, 0, 1]))).reshape(4, 4)
    A_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=30))
    B_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=30))
    threshold = 0.05
    reg_p2p = o3d.pipelines.registration.registration_icp(
        A_pcd, B_pcd, threshold, result_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    # result_T = reg_p2p.transformation
    # print(result_T)
    # print("regis inormation: ", reg_p2p.inlier_rmse)
    if SAVE_INFO:
        reg_result = icp_info.RegResult(reg_p2p)
        reg_result.compute_icp_info(A_pcd, B_pcd, True, False)
        regis_info_save_path = path+"result/trans/"+fileCur+"_"+fileNext+"_p2p_left.txt"
        reg_result.save(regis_info_save_path)
        reg_result.compute_icp_info(A_pcd, B_pcd, True, True)
        regis_info_save_path = path+"result/trans/"+fileCur+"_"+fileNext+"_p2p_right.txt"
        reg_result.save(regis_info_save_path)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        A_pcd, B_pcd, threshold, result_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    result_T = reg_p2p.transformation
    print(result_T)
    print("regis inormation: ", reg_p2p.inlier_rmse)
    if SAVE_INFO:
        reg_result = icp_info.RegResult(reg_p2p)
        reg_result.compute_icp_info(A_pcd, B_pcd, True, False)
        regis_info_save_path = path+"result/trans/"+fileCur+"_"+fileNext+"_p2s_left.txt"
        reg_result.save(regis_info_save_path)
        reg_result.compute_icp_info(A_pcd, B_pcd, True, True)
        regis_info_save_path = path+"result/trans/"+fileCur+"_"+fileNext+"_p2s_right.txt"
        reg_result.save(regis_info_save_path)

    ### save the transform source points and show regis_err
    # mkdir save path
    pcd_pair_save_path = path+"result/regis/PCD_pairs/"+fileCur+"_"+fileNext+"/"
    if (os.path.exists(pcd_pair_save_path)==False):
        os.makedirs(pcd_pair_save_path)
    A_pcd = copy.deepcopy(A_pcd_raw)
    o3d.io.write_point_cloud(pcd_pair_save_path+fileNext+".pcd", A_pcd)
    A_pcd.transform(result_T)
    o3d.io.write_point_cloud(pcd_pair_save_path+fileNext+"_regisd.pcd", A_pcd)
    B_pcd = copy.deepcopy(B_pcd_raw)
    o3d.io.write_point_cloud(pcd_pair_save_path+fileCur+".pcd", B_pcd)
    # save T path
    T_save_path = path+"result/trans/"+fileCur+"_"+fileNext+".txt"
    np.savetxt(T_save_path, (result_T[:3, :]).flatten().reshape(1, -1), delimiter=' ', fmt='%.10f')
    # save rmse path
    RMSE_save_path = path+"result/trans/"+fileCur+"_"+fileNext+"_rmse.txt"
    np.savetxt(RMSE_save_path, np.array([reg_p2p.inlier_rmse]), delimiter=' ', fmt='%.10f')
    return reg_p2p.inlier_rmse

def pgo(path, filename_list, loop_index_list):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return

    # 创建一个因子图
    graph = gtsam.NonlinearFactorGraph()
    # 创建一个初始的位姿估计
    initial_estimate = gtsam.Values()

    # 读取文件中的相邻位姿并将其添加到因子图中
    # 1. 添加位姿节点和观测
    key = []
    for i in np.arange(0, len(filename_list)):
        fileCur = filename_list[i]
        key.append(gtsam.symbol('x', fileCur))
        # 初始化位置和姿态为单位矩阵4*4
        initial_estimate.insert(key[i], gtsam.Pose3(np.eye(4)))

    # 2. 添加先验约束和相邻位姿约束
    # 添加prior factor
    odomNoiseVector6 = np.array([1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12])
    noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
    graph.add(gtsam.PriorFactorPose3(key[0], gtsam.Pose3(np.eye(4)), noise_model))
    # 添加相邻位姿约束
    for i in np.arange(0, len(filename_list)-1):
        fileCur = filename_list[i]
        fileNext = filename_list[i+1]
        T_relative_save_path = path+"result/trans/"+str(fileCur)+"_"+str(fileNext)+".txt"
        T_relative = np.loadtxt(T_relative_save_path)
        T_relative.reshape(3, 4)
        T_relative = np.hstack((T_relative, np.array([0, 0, 0, 1]))).reshape(4, 4)
        RMSE_save_path = path+"result/trans/"+str(fileCur)+"_"+str(fileNext)+"_rmse.txt"
        rmse_tmp = np.loadtxt(RMSE_save_path)
        # noise_model = gtsam.noiseModel.Isotropic.Variance(6, 0.5)
        odomNoiseVector6 = np.array([1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3])
        odomNoiseVector6 = rmse_tmp*odomNoiseVector6
        noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
        graph.add(gtsam.BetweenFactorPose3(key[i], key[i+1], gtsam.Pose3(T_relative), noise_model))

    # 3. 添加闭环约束
    T_lc = np.eye(4)
    # noise_model = gtsam.noiseModel.Isotropic.Variance(6, 0.5)
    odomNoiseVector6 = np.array([1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3])
    odomNoiseVector6 = 0.05*odomNoiseVector6
    noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
    for i in range(len(loop_index_list)):
        SeqCur = filename_list[loop_index_list[i][0]]
        SeqLC = filename_list[loop_index_list[i][1]]
        test_compute_lc_pose_diff(path, filename_list, SeqCur, SeqLC)
        graph.add(gtsam.BetweenFactorPose3(key[loop_index_list[i][0]], key[loop_index_list[i][1]], gtsam.Pose3(T_lc), noise_model))
    # # project1
    # graph.add(gtsam.BetweenFactorPose3(key[9], key[60], gtsam.Pose3(T_lc), noise_model))
    # graph.add(gtsam.BetweenFactorPose3(key[9], key[59], gtsam.Pose3(T_lc), noise_model))
    # graph.add(gtsam.BetweenFactorPose3(key[8], key[60], gtsam.Pose3(T_lc), noise_model))
    # # project2
    # T_lc = [[ 9.99999856e-01, -3.59719349e-04, -3.98348280e-04,  3.78698882e-01],
    #         [ 3.59692769e-04,  9.99999933e-01, -6.67961378e-05, -8.05217088e-02],
    #         [ 3.98372281e-04,  6.66528451e-05,  9.99999918e-01, -6.40773959e-03],
    #         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    # graph.add(gtsam.BetweenFactorPose3(key[2], key[21], gtsam.Pose3(T_lc), noise_model))

    # 创建优化问题
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    # 运行优化
    result = optimizer.optimize()

    # 输出优化结果
    optimized_pose = []
    for i in np.arange(0, len(filename_list)):
        fileCur = filename_list[i]
        optimized_pose.append(result.atPose3(key[i]))
        print('Init result:\n{}'.format(initial_estimate.atPose3(key[i])))
        print('Optimized result:\n{}'.format(result.atPose3(key[i])))

    # save result
    for i in np.arange(0, len(filename_list)):
        trans_pgo_path = path+"result/trans/absolute_pgo/"+str(filename_list[i])+".txt"
        T_absolute = optimized_pose[i].matrix()
        np.savetxt(trans_pgo_path, (T_absolute[:3, :]).flatten().reshape(1, -1), delimiter=' ', fmt='%.10f')

def compute_pose_diff(path, result1_file, result2_file, filename_list):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
    
    angle_diff_group = []
    dist_diff_group = []
    # path0_1 = path+result1_file+"/trans/absolute_pgo/"+str(filename_list[0])+".txt"
    # T0_1 = np.loadtxt(path0_1)
    # T0_1.reshape(3, 4)
    # T0_1 = np.hstack((T0_1, np.array([0, 0, 0, 1]))).reshape(4, 4)
    # path0_2 = path+result2_file+"/trans/absolute_pgo/"+str(filename_list[0])+".txt"
    # T0_2 = np.loadtxt(path0_2)
    # T0_2.reshape(3, 4)
    # T0_2 = np.hstack((T0_2, np.array([0, 0, 0, 1]))).reshape(4, 4)
    for i in np.arange(0, len(filename_list)-1):
        fileCur = filename_list[i]
        fileNext = filename_list[i+1]

        pathA = path+result1_file+"/trans/absolute_pgo/"+str(fileCur)+".txt"
        T_a = np.loadtxt(pathA)
        T_a.reshape(3, 4)
        T_a = np.hstack((T_a, np.array([0, 0, 0, 1]))).reshape(4, 4)
        pathB = path+result1_file+"/trans/absolute_pgo/"+str(fileNext)+".txt"
        T_b = np.loadtxt(pathB)
        T_b.reshape(3, 4)
        T_b = np.hstack((T_b, np.array([0, 0, 0, 1]))).reshape(4, 4)
        T_relative1 = np.dot(np.linalg.inv(T_a), T_b)

        pathA = path+result2_file+"/trans/absolute_pgo/"+str(fileCur)+".txt"
        T_a = np.loadtxt(pathA)
        T_a.reshape(3, 4)
        T_a = np.hstack((T_a, np.array([0, 0, 0, 1]))).reshape(4, 4)
        pathB = path+result2_file+"/trans/absolute_pgo/"+str(fileNext)+".txt"
        T_b = np.loadtxt(pathB)
        T_b.reshape(3, 4)
        T_b = np.hstack((T_b, np.array([0, 0, 0, 1]))).reshape(4, 4)
        T_relative2 = np.dot(np.linalg.inv(T_a), T_b)

        R_diff = np.dot(np.linalg.inv(T_relative1[:3, :3]), T_relative2[:3, :3])
        # 正交化
        U, D, Vt = svd(R_diff)
        R_diff = np.dot(U, Vt)
        # 计算旋转矩阵的行列式
        det_R = np.linalg.det(R_diff)
        # 计算行列式的三次方根
        cbrt_det_R = abs(det_R) ** (1/3)
        # 归一化旋转矩阵
        R_diff = R_diff / cbrt_det_R
        # 确保列向量是单位向量
        for i in range(3):
            R_diff[:, i] /= np.linalg.norm(R_diff[:, i])

        t_diff = T_relative2[:3, 3] - T_relative1[:3, 3]
        angle_diff = np.arccos((np.trace(R_diff)-1)/2)
        dist_diff = np.sqrt(np.sum(np.square(t_diff)))
        angle_diff_group.append(angle_diff)
        dist_diff_group.append(dist_diff)
        print(fileNext, ":")
        print("(np.trace(R_diff)-1)/2: ", (np.trace(R_diff)-1)/2)
        print("R_diff: ", R_diff)
        print("angle_diff: ", angle_diff*180/np.pi, "deg")
        print("dist_diff: ", dist_diff, "m")

    # draw angle_diff and dist_diff
    # 创建图形对象
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ax1.plot(filename_list[1:], angle_diff_group, 'r', label='angle_diff')
    ax2.plot(filename_list[1:], dist_diff_group, 'b', label='dist_diff')
    ax1.legend()
    ax2.legend()
    plt.show()

def test_compute_lc_pose_diff(path, filename_list, SeqCur, SeqLC):
    if (os.path.exists(path)==False):
        print(path+"NOT FIND!!!")
        return
    if SeqCur>SeqLC:
        tmp = SeqCur
        SeqCur = SeqLC
        SeqLC = tmp
    T1 = np.eye(4)
    for i in np.arange(SeqCur-1, SeqLC-1):
        fileCur = str(filename_list[i])
        fileNext = str(filename_list[i+1])
        # load relative trans from txt
        T_relative_save_path = path+"result/trans/"+fileCur+"_"+fileNext+".txt"
        T_relative = np.loadtxt(T_relative_save_path)
        T_relative.reshape(3, 4)
        T_relative = np.hstack((T_relative, np.array([0, 0, 0, 1]))).reshape(4, 4)
        T1 = np.dot(T1, T_relative)
    T_LC_save_path = path+"result/trans/"+str(SeqCur)+"_"+str(SeqLC)+".txt"
    T2 = np.loadtxt(T_LC_save_path)
    T2.reshape(3, 4)
    T2 = np.hstack((T2, np.array([0, 0, 0, 1]))).reshape(4, 4)

    T_af_pgo_cur_path = path+"result/trans/absolute_pgo/"+str(SeqCur)+".txt"
    T_cur = np.loadtxt(T_af_pgo_cur_path)
    T_cur.reshape(3, 4)
    T_cur = np.hstack((T_cur, np.array([0, 0, 0, 1]))).reshape(4, 4)
    T_af_pgo_lc_path = path+"result/trans/absolute_pgo/"+str(SeqLC)+".txt"
    T_lc = np.loadtxt(T_af_pgo_lc_path)
    T_lc.reshape(3, 4)
    T_lc = np.hstack((T_lc, np.array([0, 0, 0, 1]))).reshape(4, 4)
    T3 = np.dot(np.linalg.inv(T_cur), T_lc)

    # before pgo #############################
    T_diff = np.dot(np.linalg.inv(T1), T2)
    R_diff = T_diff[:3, :3]
    # 正交化
    U, D, Vt = svd(R_diff)
    R_diff = np.dot(U, Vt)
    # 计算旋转矩阵的行列式
    det_R = np.linalg.det(R_diff)
    # 计算行列式的三次方根
    cbrt_det_R = abs(det_R) ** (1/3)
    # 归一化旋转矩阵
    R_diff = R_diff / cbrt_det_R
    # 确保列向量是单位向量
    for i in range(3):
        R_diff[:, i] /= np.linalg.norm(R_diff[:, i])

    t_diff = T_diff[:3, 3]
    angle_diff = np.arccos((np.trace(R_diff)-1)/2)
    dist_diff = np.sqrt(np.sum(np.square(t_diff)))
    print("pose loop diff between", SeqCur, "and", SeqLC, "before pgo:")
    print("T_diff:\n", T_diff)
    # print("(np.trace(R_diff)-1)/2: ", (np.trace(R_diff)-1)/2)
    # print("R_diff: ", R_diff)
    print("angle_diff: ", angle_diff*180/np.pi, "deg")
    print("dist_diff: ", dist_diff, "m")

    # after pgo #################################
    T_diff = np.dot(np.linalg.inv(T3), T2)
    R_diff = T_diff[:3, :3]
    # 正交化
    U, D, Vt = svd(R_diff)
    R_diff = np.dot(U, Vt)
    # 计算旋转矩阵的行列式
    det_R = np.linalg.det(R_diff)
    # 计算行列式的三次方根
    cbrt_det_R = abs(det_R) ** (1/3)
    # 归一化旋转矩阵
    R_diff = R_diff / cbrt_det_R
    # 确保列向量是单位向量
    for i in range(3):
        R_diff[:, i] /= np.linalg.norm(R_diff[:, i])

    t_diff = T_diff[:3, 3]
    angle_diff = np.arccos((np.trace(R_diff)-1)/2)
    dist_diff = np.sqrt(np.sum(np.square(t_diff)))
    print("pose loop diff between", SeqCur, "and", SeqLC, "after pgo:")
    print("T_diff:\n", T_diff)
    # print("(np.trace(R_diff)-1)/2: ", (np.trace(R_diff)-1)/2)
    # print("R_diff: ", R_diff)
    print("angle_diff: ", angle_diff*180/np.pi, "deg")
    print("dist_diff: ", dist_diff, "m")

def seq_regis_main(path, filename_list, loop_index_list):
    ## read && downsample 
    ### 1. las to pcd (include downsample 1/10) 
    ### 2. downsample(use voxelsize=0.05)
    # data_down_sample(path, filename_list)

    ## choice 1: solve all
    # whutls_compute_trans(path, filename_list) 

    # ## choice 2: solve one selected
    # # filename_list = np.array([2, 3, 7, 11, 21, 23, 24, 25, 28, 31, 32, 
    # #                           33, 35, 41, 45, 49, 50, 51, 53, 56, 59])
    # inlier_rmse_group = []
    # for i in np.arange(0, len(filename_list)-1):
    #     SeqCur = filename_list[i]
    #     SeqNext = SeqCur+1
    #     inlier_rmse = whutls_one2one_match(path, filename_list, SeqCur, SeqNext)
    #     inlier_rmse_group.append(inlier_rmse)
    
    # SeqCur = 10
    # SeqNext = 61
    # whutls_one2one_match(path, filename_list, SeqCur, SeqNext)

    # pgo(path, filename_list, loop_index_list)

    # whutls_regis_from_absolute_trans(path, filename_list)
    # whutls_regis_from_relative_trans(path, filename_list)

    # compute_pose_diff(path, "project1_result_1015", "result", filename_list)

    for i in range(len(loop_index_list)):
        SeqCur = filename_list[loop_index_list[i][0]]
        SeqLC = filename_list[loop_index_list[i][1]]
        test_compute_lc_pose_diff(path, filename_list, SeqCur, SeqLC)

if __name__ == '__main__':
    # path = "/media/wbl/KESU1/data/whu_tls/project1/"
    # filename_list = np.arange(1, 62)
    # loop_index_list = [[9, 60], [9, 59], [8, 60]]
    # seq_regis_main(path, filename_list, loop_index_list)

    path = "/media/wbl/KESU1/data/whu_tls/project2/"
    filename_list = np.arange(1, 33)
    loop_index_list = [[2, 21]]
    seq_regis_main(path, filename_list, loop_index_list)