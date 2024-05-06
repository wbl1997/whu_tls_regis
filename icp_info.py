"""
Compute ICP information matrix.
"""
import numpy as np
import open3d as o3d

def skew(v):
    """v is a 3d vector"""
    return np.array([[0, -v[2, 0], v[1, 0]],
                     [v[2, 0], 0, -v[0, 0]],
                     [-v[1, 0], v[0, 0], 0]])


class RegResult(object):
    def __init__(self, reg_res):
        """reg_res is the output of o3d.registration.registration_icp"""
        self.reg_res = reg_res
        self.info_mat = None
        self.cov_mat = None
        self.rightinvarianterror = False

    def save(self, filename):
        with open(filename, 'w') as stream:
            np_corr_set = np.asarray(self.reg_res.correspondence_set)
            n = np_corr_set.shape[0]
            stream.write("#correspondence_set: {}\n".format(n))
            stream.write("fitness: {:.6f}\n".format(self.reg_res.fitness))
            stream.write("inlier_rmse: {:.6f}\n".format(self.reg_res.inlier_rmse))
            stream.write("transformation:\n")
            stream.write("{}\n".format(self.reg_res.transformation))
            stream.write("covariance:\n")
            stream.write("{}\n".format(self.cov_mat))
            stream.write("rightinvarianterror: {}\n".format(self.rightinvarianterror))

    def compute_icp_info(self, source_pcd, target_pcd, pointtoplane, rightinvarianterror):
        """
        The right invariant error for rotation is defined by exp(\theta) t_\hat{R}_s = t_R_s
        gtsam uses left invariant error for rotations, ie, t_\hat{R}_s exp(\theta) = t_R_s
        For translation, both left and right invariant errors are the same, ie, t_\hat{t}_s + \delta t = t_t_s. 
        """
        src_array = np.asarray(source_pcd.points)
        np_corr_set = np.asarray(self.reg_res.correspondence_set)
        n = np_corr_set.shape[0]
        if pointtoplane:
            tgt_normals = np.asarray(target_pcd.normals)
            A = np.zeros((n, 6))
            if rightinvarianterror:
                for i in range(n):
                    src_id = np_corr_set[i, 0]
                    tgt_id = np_corr_set[i, 1]
                    t_R_s = self.reg_res.transformation[0 : 3, 0 : 3]
                    src_point = src_array[src_id, :].reshape(3, 1)
                    rot_src_point = np.matmul(t_R_s, src_point)
                    skew_src_point = skew(rot_src_point)
                    A[i, 0 : 3] = tgt_normals[tgt_id, :]
                    A[i, 3 : 6] = -np.matmul(tgt_normals[tgt_id, :], skew_src_point)
            else:
                for i in range(n):
                    src_id = np_corr_set[i, 0]
                    tgt_id = np_corr_set[i, 1]
                    t_R_s = self.reg_res.transformation[0 : 3, 0 : 3]
                    src_point = src_array[src_id, :].reshape(3, 1)
                    skew_src_point = skew(src_point)
                    rot_src_point = np.matmul(t_R_s, skew_src_point)
                    A[i, 0 : 3] = tgt_normals[tgt_id, :]
                    A[i, 3 : 6] = -np.matmul(tgt_normals[tgt_id, :], rot_src_point)
        else:
            A = np.zeros((n * 3, 6))
            if rightinvarianterror:
                for i in range(n):
                    src_id = np_corr_set[i, 0]
                    tgt_id = np_corr_set[i, 1]
                    t_R_s = self.reg_res.transformation[0 : 3, 0 : 3]
                    src_point = src_array[src_id, :].reshape(3, 1)
                    rot_src_point = np.matmul(t_R_s, src_point)
                    skew_src_point = skew(rot_src_point)
                    A[i * 3 : i * 3 + 3, 0 : 3] = np.identity(3)
                    A[i * 3 : i * 3 + 3, 3 : 6] = -skew_src_point
            else:
                for i in range(n):
                    src_id = np_corr_set[i, 0]
                    tgt_id = np_corr_set[i, 1]
                    t_R_s = self.reg_res.transformation[0 : 3, 0 : 3]
                    src_point = src_array[src_id, :].reshape(3, 1)
                    skew_src_point = skew(src_point)
                    rot_src_point = np.matmul(t_R_s, skew_src_point)
                    A[i * 3 : i * 3 + 3, 0 : 3] = np.identity(3)
                    A[i * 3 : i * 3 + 3, 3 : 6] = -rot_src_point
        self.info_mat = np.matmul(A.T, A)
        self.cov_mat = np.linalg.inv(self.info_mat)
        self.rightinvarianterror = rightinvarianterror


