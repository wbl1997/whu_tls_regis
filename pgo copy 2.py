import os
import numpy as np
import gtsam
from scipy.spatial.transform import Slerp, Rotation as R

def load_tum_poses(file_path):
    poses = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.strip().split()
            timestamp = float(data[0])
            tx, ty, tz = map(float, data[1:4])
            qx, qy, qz, qw = map(float, data[4:8])
            pose = gtsam.Pose3(gtsam.Rot3.Quaternion(qw, qx, qy, qz), gtsam.Point3(tx, ty, tz))
            poses.append((timestamp, pose))
    return poses

def save_poses_to_tum(file_path, poses):
    with open(file_path, 'w') as f:
        for timestamp, pose in poses:
            translation = pose.translation()
            rotation = pose.rotation().toQuaternion()
            f.write(f"{timestamp:.6f} {translation[0]} {translation[1]} {translation[2]} {rotation.x()} {rotation.y()} {rotation.z()} {rotation.w()}\n")

def interpolate_pose(pose1, pose2, t):
    """Interpolate between two poses."""
    t1, p1 = pose1
    t2, p2 = pose2
    alpha = (t - t1) / (t2 - t1)
    
    pos1 = p1.translation()
    pos2 = p2.translation()
    trans = (1 - alpha) * pos1 + alpha * pos2
    
    rot1 = p1.rotation().toQuaternion()
    rot2 = p2.rotation().toQuaternion()
    rot1 = np.array([rot1.x(), rot1.y(), rot1.z(), rot1.w()])
    rot2 = np.array([rot2.x(), rot2.y(), rot2.z(), rot2.w()])
    
    slerp = Slerp([t1, t2], R.from_quat([rot1, rot2]))
    rot = slerp(t).as_quat()
    
    return gtsam.Pose3(gtsam.Rot3.Quaternion(rot[3], rot[0], rot[1], rot[2]), gtsam.Point3(trans))

def interpolate_absolute_poses(relative_poses, absolute_poses):
    interpolated_poses = []
    absolute_timestamps = [pose[0] for pose in absolute_poses]
    overlap_mask = []

    for timestamp, _ in relative_poses:
        if timestamp <= absolute_timestamps[0]:
            interpolated_poses.append((timestamp, absolute_poses[0][1]))
            overlap_mask.append(0)
        elif timestamp >= absolute_timestamps[-1]:
            interpolated_poses.append((timestamp, absolute_poses[-1][1]))
            overlap_mask.append(0)
        else:
            for i in range(len(absolute_poses) - 1):
                if absolute_timestamps[i] <= timestamp <= absolute_timestamps[i + 1]:
                    if np.abs(absolute_timestamps[i] - timestamp) > 2 or np.abs(absolute_timestamps[i + 1] - timestamp) > 2:
                        interpolated_poses.append((timestamp, interpolate_pose(absolute_poses[i], absolute_poses[i + 1], timestamp)))
                        overlap_mask.append(0)
                    else:
                        interpolated_poses.append((timestamp, interpolate_pose(absolute_poses[i], absolute_poses[i + 1], timestamp)))
                        overlap_mask.append(1)
                    break
    
    return interpolated_poses, overlap_mask

def pgo(relative_pose_file, absolute_pose_file, loop_index_list = [], output_file="optimized_poses.txt"):
    if not os.path.exists(relative_pose_file) or not os.path.exists(absolute_pose_file):
        print("File(s) not found!")
        return

    # 加载位姿文件
    relative_poses = load_tum_poses(relative_pose_file)
    absolute_poses = load_tum_poses(absolute_pose_file)

    # 对齐并插值绝对位姿
    interpolated_absolute_poses, overlap_mask = interpolate_absolute_poses(relative_poses, absolute_poses)
    print(overlap_mask)

    # 创建一个因子图
    graph = gtsam.NonlinearFactorGraph()
    # 创建一个初始的位姿估计
    initial_estimate = gtsam.Values()

    # 添加位姿节点和观测
    key = []
    for i, (timestamp, pose) in enumerate(relative_poses):
        key.append(gtsam.symbol('x', i))
        initial_estimate.insert(key[i], interpolated_absolute_poses[i][1])

    # 添加先验约束
    odomNoiseVector6 = np.array([1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12])
    noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
    graph.add(gtsam.PriorFactorPose3(key[0], interpolated_absolute_poses[0][1], noise_model))

    # 添加相邻位姿约束
    for i in range(len(relative_poses) - 1):
        T_relative = relative_poses[i][1].between(relative_poses[i + 1][1])
        odomNoiseVector6 = np.array([1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3])
        noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
        graph.add(gtsam.BetweenFactorPose3(key[i], key[i + 1], T_relative, noise_model))

    for i in range(len(relative_poses)-1):
        if overlap_mask[i] == 1 and overlap_mask[i+1] == 1:
            noise_model = gtsam.noiseModel.Diagonal.Variances(np.array([1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]))
            graph.add(gtsam.BetweenFactorPose3(key[i+1], key[i], interpolated_absolute_poses[i+1][1].between(interpolated_absolute_poses[i][1]), noise_model))

    # # add absolute pose constraints
    # for i in range(len(relative_poses)):
    #     if overlap_mask[i] != 1:
    #         continue
    #     noise_model = gtsam.noiseModel.Diagonal.Variances(np.array([1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]))
    #     graph.add(gtsam.BetweenFactorPose3(key[i], key[i], interpolated_absolute_poses[i][1].between(interpolated_absolute_poses[i][1]), noise_model))

    # 添加闭环约束（位置使用内插的相对位置，姿态使用相对位姿）
    odomNoiseVector6 = np.array([1e-1, 1e-1, 1e-1, 1e-3, 1e-3, 1e-3])
    noise_model = gtsam.noiseModel.Diagonal.Variances(odomNoiseVector6)
    # for loop_index in loop_index_list:
    #     # pos1 = interpolated_absolute_poses[loop_index[0]][1].translation()
    #     # pos2 = interpolated_absolute_poses[loop_index[1]][1].translation()
    #     # pos_lc = gtsam.Point3(pos2 - pos1)
    #     # rot_lc = relative_poses[loop_index[0]][1].rotation().between(relative_poses[loop_index[1]][1].rotation())
    #     # T_lc = gtsam.Pose3(rot_lc, pos_lc)
    #     T_lc = relative_poses[loop_index[1]][1].between(relative_poses[loop_index[0]][1])                   
    #     graph.add(gtsam.BetweenFactorPose3(key[loop_index[1]], key[loop_index[0]], T_lc, noise_model))
    last_idx = len(relative_poses) - 1
    # T_lc = relative_poses[0][1].between(relative_poses[last_idx][1])
    # graph.add(gtsam.BetweenFactorPose3(key[0], key[last_idx], T_lc, noise_model))
    T_lc = interpolated_absolute_poses[last_idx][1].between(interpolated_absolute_poses[0][1])
    print("T_lc: ", T_lc)
    graph.add(gtsam.BetweenFactorPose3(key[last_idx], key[0], T_lc, noise_model))

    print("len(relative_poses): ", len(relative_poses))

    # 创建优化问题
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    # 运行优化
    result = optimizer.optimize()

    # 输出优化结果
    optimized_pose = []
    for i in range(len(relative_poses)):
        optimized_pose.append((relative_poses[i][0], result.atPose3(key[i])))
        # print(f'Init result:\n{initial_estimate.atPose3(key[i])}')
        # print(f'Optimized result:\n{result.atPose3(key[i])}')

    # print optimized poses between last_idx and 0
    last_pose = result.atPose3(key[last_idx])
    first_pose = result.atPose3(key[0])
    T_lc = last_pose.between(first_pose)
    print("T_lc: ", T_lc)

    # 保存结果到文件
    save_poses_to_tum(output_file, optimized_pose)
    import matplotlib.pyplot as plt
    optimized_poses = [result.atPose3(key[i]) for i in range(len(relative_poses))]
    optimized_poses = np.array([pose.translation() for pose in optimized_poses])
    plt.plot(optimized_poses[:, 0], optimized_poses[:, 1], 'r-')
    plt.show()  

# 示例调用
relative_pose_file = "/home/wbl/code/visual_radar_slam/algorithm_result/in_house/20240113_data3_aligned/FastLIO2.txt"
absolute_pose_file = "/home/wbl/code/visual_radar_slam/algorithm_result/in_house/20240113_data3_aligned/groundtruth.txt"
loop_index_list = [(0, 100)]
pgo(relative_pose_file, absolute_pose_file, loop_index_list)
