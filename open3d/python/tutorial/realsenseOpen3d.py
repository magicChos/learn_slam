#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :realsenseOpen3d.py
@brief       :RealSense with Open3D
@time        :2021/06/25 16:04:03
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import open3d as o3d
import numpy as np
from os.path import join, dirname, basename, splitext, isfile
from warnings import warn
import json

bag_filename = "/home/han/Desktop/data_0709/20210712_091914.bag"
config_file = "/home/han/project/learn_slam/open3d/python/tutorial/realsense.json"
color_file = "/home/han/Desktop/data_0709/20210712_091914/color/00000.jpg"
depth_file = "/home/han/Desktop/data_0709/20210712_091914/depth/00000.png"


def set_default_value(config, key, value):
    if key not in config:
        config[key] = value


def extract_rgbd_frames(rgbd_video_file):
    """
    Extract color and aligned depth frames and intrinsic calibration from an
    RGBD video file (currently only RealSense bag files supported). Folder
    structure is:
        <directory of rgbd_video_file/<rgbd_video_file name without extension>/
            {depth/00000.jpg,color/00000.png,intrinsic.json}
    """
    frames_folder = join(dirname(rgbd_video_file),
                         basename(splitext(rgbd_video_file)[0]))
    path_intrinsic = join(frames_folder, "intrinsic.json")
    if isfile(path_intrinsic):
        warn(f"Skipping frame extraction for {rgbd_video_file} since files are"
             " present.")
    else:
        rgbd_video = o3d.t.io.RGBDVideoReader.create(rgbd_video_file)
        rgbd_video.save_frames(frames_folder)
    with open(path_intrinsic) as intr_file:
        intr = json.load(intr_file)
    depth_scale = intr["depth_scale"]
    return frames_folder, path_intrinsic, depth_scale


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


def createMesh(color_file, depth_file, intrinsic, config):
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=3.0 / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    rgbd = read_rgbd_image(color_file, depth_file, False, config)
    volume.integrate(rgbd, intrinsic, np.identity(4))
    
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, intrinsic)

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    
    return mesh , pcd


def read_json(json_path):
    info = None
    with open(json_path, 'r') as f:
        info = json.load(f)
    return info


def main():
    config = read_json(config_file)
    frames_folder, path_intrinsic, depth_scale = extract_rgbd_frames(bag_filename)
    config["path_intrinsic"] = path_intrinsic
    config["depth_scale"] = depth_scale
    intrinsic = o3d.io.read_pinhole_camera_intrinsic(config["path_intrinsic"])
    

    mesh , pcd = createMesh(color_file , depth_file , intrinsic , config)
    o3d.io.write_triangle_mesh("p_mesh_c.ply", mesh)
    o3d.io.write_point_cloud("pcd.ply", pcd, False, True)


if __name__ == '__main__':
    main()
