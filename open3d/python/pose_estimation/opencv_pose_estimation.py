#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :opencv_pose_estimation.py
@brief       :通过两帧RGBD图像估计位姿关系
@time        :2021/06/28 16:00:21
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


# examples/python/reconstruction_system/opencv_pose_estimation.py


import numpy as np
import cv2
from matplotlib import pyplot as plt  # for visualizing feature matching
import copy
import open3d as o3d
import json

# source_rgbd_image: open3d.geometry.RGBDImage
# target_rgbd_image: open3d.geometry.RGBDImage
# pinhole_camera_intrinsic: open3d.camera.PinholeCameraIntrinsic


def pose_estimation(source_rgbd_image, target_rgbd_image,
                    pinhole_camera_intrinsic, debug_draw_correspondences):
    success = False
    trans = np.identity(4)

    # transform double array to unit8 array
    color_cv_s = np.uint8(np.asarray(source_rgbd_image.color) * 255.0)
    color_cv_t = np.uint8(np.asarray(target_rgbd_image.color) * 255.0)
    
    cv2.imshow("color_cv_s" , color_cv_s)
    cv2.imshow("color_cv_t" , color_cv_t)
    cv2.waitKey(0)

    orb = cv2.ORB_create(scaleFactor=1.2,
                         nlevels=8,
                         edgeThreshold=31,
                         firstLevel=0,
                         WTA_K=2,
                         scoreType=cv2.ORB_HARRIS_SCORE,
                         nfeatures=100,
                         patchSize=31)  # to save time
    [kp_s, des_s] = orb.detectAndCompute(color_cv_s, None)
    [kp_t, des_t] = orb.detectAndCompute(color_cv_t, None)
    if len(kp_s) == 0 or len(kp_t) == 0:
        return success, trans

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des_s, des_t)

    pts_s = []
    pts_t = []
    for match in matches:
        pts_t.append(kp_t[match.trainIdx].pt)
        pts_s.append(kp_s[match.queryIdx].pt)
    pts_s = np.asarray(pts_s)
    pts_t = np.asarray(pts_t)
    # inlier points after initial BF matching
    if debug_draw_correspondences:
        draw_correspondences(np.asarray(source_rgbd_image.color),
                             np.asarray(target_rgbd_image.color), pts_s, pts_t,
                             np.ones(pts_s.shape[0]), "Initial BF matching")

    focal_input = (pinhole_camera_intrinsic.intrinsic_matrix[0, 0] +
                   pinhole_camera_intrinsic.intrinsic_matrix[1, 1]) / 2.0
    pp_x = pinhole_camera_intrinsic.intrinsic_matrix[0, 2]
    pp_y = pinhole_camera_intrinsic.intrinsic_matrix[1, 2]

    # Essential matrix is made for masking inliers
    pts_s_int = np.int32(pts_s + 0.5)
    pts_t_int = np.int32(pts_t + 0.5)
    [E, mask] = cv2.findEssentialMat(pts_s_int,
                                     pts_t_int,
                                     focal=focal_input,
                                     pp=(pp_x, pp_y),
                                     method=cv2.RANSAC,
                                     prob=0.999,
                                     threshold=1.0)
    if mask is None:
        return success, trans

    # inlier points after 5pt algorithm
    if debug_draw_correspondences:
        draw_correspondences(np.asarray(source_rgbd_image.color),
                             np.asarray(target_rgbd_image.color), pts_s, pts_t,
                             mask, "5-pt RANSAC")

    # make 3D correspondences
    depth_s = np.asarray(source_rgbd_image.depth)
    depth_t = np.asarray(target_rgbd_image.depth)
    pts_xyz_s = np.zeros([3, pts_s.shape[0]])
    pts_xyz_t = np.zeros([3, pts_s.shape[0]])
    cnt = 0
    for i in range(pts_s.shape[0]):
        if mask[i]:
            xyz_s = get_xyz_from_pts(pts_s[i, :], depth_s, pp_x, pp_y,
                                     focal_input)
            pts_xyz_s[:, cnt] = xyz_s
            xyz_t = get_xyz_from_pts(pts_t[i, :], depth_t, pp_x, pp_y,
                                     focal_input)
            pts_xyz_t[:, cnt] = xyz_t
            cnt = cnt + 1
    pts_xyz_s = pts_xyz_s[:, :cnt]
    pts_xyz_t = pts_xyz_t[:, :cnt]

    success, trans, inlier_id_vec = estimate_3D_transform_RANSAC(
        pts_xyz_s, pts_xyz_t)

    if debug_draw_correspondences:
        pts_s_new = np.zeros(shape=(len(inlier_id_vec), 2))
        pts_t_new = np.zeros(shape=(len(inlier_id_vec), 2))
        mask = np.ones(len(inlier_id_vec))
        cnt = 0
        for i in inlier_id_vec:
            u_s, v_s = get_uv_from_xyz(pts_xyz_s[0, i], pts_xyz_s[1, i],
                                       pts_xyz_s[2, i], pp_x, pp_y, focal_input)
            u_t, v_t = get_uv_from_xyz(pts_xyz_t[0, i], pts_xyz_t[1, i],
                                       pts_xyz_t[2, i], pp_x, pp_y, focal_input)
            pts_s_new[cnt, :] = [u_s, v_s]
            pts_t_new[cnt, :] = [u_t, v_t]
            cnt = cnt + 1
        draw_correspondences(np.asarray(source_rgbd_image.color),
                             np.asarray(target_rgbd_image.color), pts_s_new,
                             pts_t_new, mask, "5-pt RANSAC + 3D Rigid RANSAC")
    return success, trans


def draw_correspondences(img_s, img_t, pts_s, pts_t, mask, title):
    ha, wa = img_s.shape[:2]
    hb, wb = img_t.shape[:2]
    total_width = wa + wb
    new_img = np.zeros(shape=(ha, total_width , 3))
    
    # import pdb;pdb.set_trace()
    new_img[:ha, :wa] = img_s
    new_img[:hb, wa:wa + wb] = img_t

    fig = plt.figure()
    fig.canvas.set_window_title(title)
    for i in range(pts_s.shape[0]):
        if mask[i]:
            sx = pts_s[i, 0]
            sy = pts_s[i, 1]
            tx = pts_t[i, 0] + wa
            ty = pts_t[i, 1]
            plt.plot([sx, tx], [sy, ty],
                     color=np.random.random(3) / 2 + 0.5,
                     lw=1.0)
    plt.imshow(new_img)
    plt.show()
    # plt.pause(0.5)
    # plt.close()


def estimate_3D_transform_RANSAC(pts_xyz_s, pts_xyz_t):
    max_iter = 1000
    max_distance = 0.05
    n_sample = 5
    n_points = pts_xyz_s.shape[1]
    Transform_good = np.identity(4)
    max_inlier = n_sample
    inlier_vec_good = []
    success = False

    if n_points < n_sample:
        return False, np.identity(4), []

    for i in range(max_iter):

        # sampling
        rand_idx = np.random.randint(n_points, size=n_sample)
        sample_xyz_s = pts_xyz_s[:, rand_idx]
        sample_xyz_t = pts_xyz_t[:, rand_idx]
        R_approx, t_approx = estimate_3D_transform(sample_xyz_s, sample_xyz_t)

        # evaluation
        diff_mat = pts_xyz_t - (np.matmul(R_approx, pts_xyz_s) +
                                np.tile(t_approx, [1, n_points]))
        diff = [np.linalg.norm(diff_mat[:, i]) for i in range(n_points)]
        n_inlier = len([1 for diff_iter in diff if diff_iter < max_distance])

        # note: diag(R_approx) > 0 prevents ankward transformation between
        # RGBD pair of relatively small amount of baseline.
        if (n_inlier > max_inlier) and (np.linalg.det(R_approx) != 0.0) and \
                (R_approx[0, 0] > 0 and R_approx[1, 1] > 0 and R_approx[2, 2] > 0):
            Transform_good[:3, :3] = R_approx
            Transform_good[:3, 3] = [t_approx[0], t_approx[1], t_approx[2]]
            max_inlier = n_inlier
            inlier_vec = [id_iter for diff_iter, id_iter
                          in zip(diff, range(n_points))
                          if diff_iter < max_distance]
            inlier_vec_good = inlier_vec
            success = True

    return success, Transform_good, inlier_vec_good


# singular value decomposition approach
# based on the description in the sec 3.1.2 in
# http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
def estimate_3D_transform(input_xyz_s, input_xyz_t):
    # compute H
    xyz_s = copy.copy(input_xyz_s)
    xyz_t = copy.copy(input_xyz_t)
    n_points = xyz_s.shape[1]
    mean_s = np.mean(xyz_s, axis=1)
    mean_t = np.mean(xyz_t, axis=1)
    mean_s.shape = (3, 1)
    mean_t.shape = (3, 1)
    xyz_diff_s = xyz_s - np.tile(mean_s, [1, n_points])
    xyz_diff_t = xyz_t - np.tile(mean_t, [1, n_points])
    H = np.matmul(xyz_diff_s, xyz_diff_t.transpose())
    # solve system
    U, s, V = np.linalg.svd(H)
    R_approx = np.matmul(V.transpose(), U.transpose())
    if np.linalg.det(R_approx) < 0.0:
        det = np.linalg.det(np.matmul(U, V))
        D = np.identity(3)
        D[2, 2] = det
        R_approx = np.matmul(U, np.matmul(D, V))
    t_approx = mean_t - np.matmul(R_approx, mean_s)
    return R_approx, t_approx


def get_xyz_from_pts(pts_row, depth, px, py, focal):
    u = pts_row[0]
    v = pts_row[1]
    u0 = int(u)
    v0 = int(v)
    height = depth.shape[0]
    width = depth.shape[1]
    # bilinear depth interpolation
    if u0 > 0 and u0 < width - 1 and v0 > 0 and v0 < height - 1:
        up = pts_row[0] - u0
        vp = pts_row[1] - v0
        d0 = depth[v0, u0]
        d1 = depth[v0, u0 + 1]
        d2 = depth[v0 + 1, u0]
        d3 = depth[v0 + 1, u0 + 1]
        d = (1 - vp) * (d1 * up + d0 * (1 - up)) + vp * (d3 * up + d2 *
                                                         (1 - up))
        return get_xyz_from_uv(u, v, d, px, py, focal)
    else:
        return [0, 0, 0]


def get_xyz_from_uv(u, v, d, px, py, focal):
    if focal != 0:
        x = (u - px) / focal * d
        y = (v - py) / focal * d
    else:
        x = 0
        y = 0
    return np.array([x, y, d]).transpose()


def get_uv_from_xyz(x, y, z, px, py, focal):
    if z != 0:
        u = focal * x / z + px
        v = focal * y / z + py
    else:
        u = 0
        v = 0
    return u, v


def read_rgbd_image(color_file, depth_file, convert_rgb_to_intensity, config):
    color = o3d.io.read_image(color_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=config["depth_scale"],
        depth_trunc=config["max_depth"],
        convert_rgb_to_intensity=convert_rgb_to_intensity)
    return rgbd_image

def read_json(json_path):
    info = None
    with open(json_path, 'r') as f:
        info = json.load(f)
    return info

if __name__ == '__main__':
    color_files = ["/home/han/3rdparty/datasets/L515_JackJack/color/00000.jpg" , "/home/han/3rdparty/datasets/L515_JackJack/color/00050.jpg"]
    depth_files = ["/home/han/3rdparty/datasets/L515_JackJack/depth/00000.png" , "/home/han/3rdparty/datasets/L515_JackJack/depth/00050.png"]
    config_file = "/home/han/data/project/learn_slam/open3d/python/tutorial/realsense.json"
    intrinsic_file = "/home/han/3rdparty/datasets/L515_JackJack/intrinsic.json"
    
    config = read_json(config_file)
    intrinsic = o3d.io.read_pinhole_camera_intrinsic(intrinsic_file)
    
    rgbd1 = read_rgbd_image(color_files[0] , depth_files[0] , False , config)
    rgbd2 = read_rgbd_image(color_files[1] , depth_files[1] , False , config)
    success, trans = pose_estimation(rgbd1 , rgbd2 , intrinsic , True)
    if success:
        print(trans)
     
    
    
