import icp_info
import open3d as o3d
import numpy as np

tgt_pcd = o3d.io.read_point_cloud("data/project2/result/downsample/1.pcd")
src_pcd = o3d.io.read_point_cloud("data/project2/result/downsample/2.pcd")

src_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=30))
tgt_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=30))
threshold = 0.1
tgt_T_src = np.identity(4)

reg_res = o3d.pipelines.registration.registration_icp(
    src_pcd, tgt_pcd, threshold, tgt_T_src,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
)
reg_result = icp_info.RegResult(reg_res)
reg_result.compute_icp_info(src_pcd, tgt_pcd, True, False)
reg_result.save("data/project2/result/1_2_p2s_left.txt")
reg_result.compute_icp_info(src_pcd, tgt_pcd, True, True)
reg_result.save("data/project2/result/1_2_p2s_right.txt")

reg_res = o3d.pipelines.registration.registration_icp(
    src_pcd, tgt_pcd, threshold, tgt_T_src,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
)
reg_result = icp_info.RegResult(reg_res)
rightinvarianterror = True
reg_result.compute_icp_info(src_pcd, tgt_pcd, False, False)
reg_result.save("data/project2/result/1_2_p2p_left.txt")
reg_result.compute_icp_info(src_pcd, tgt_pcd, False, True)
reg_result.save("data/project2/result/1_2_p2p_right.txt")
