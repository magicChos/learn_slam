#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :generate_meshes.py
@brief       :利用点云生成mesh
@time        :2021/06/16 13:53:29
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import pdb
import numpy as np
import open3d as o3d
import argparse
import matplotlib.pyplot as plt
import os


def ComputeTriangleArea(pt0, pt1, pt2):
    x = pt0 - pt1
    y = pt0 - pt2
    x_y = np.cross(x, y)
    area = 0.5 * np.linalg.norm(x_y)
    return area

# BPA算法


def BPA(pcd):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector([radius, radius * 2]))
    dec_mesh = bpa_mesh.simplify_quadric_decimation(1000000)

    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()

    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = dec_mesh.crop(bbox)

    lods = lod_mesh_export(
        p_mesh_crop, [1000000, 100000, 10000, 1000], ".ply", ".")
    o3d.visualization.draw_geometries([lods[1000000]])
    o3d.io.write_triangle_mesh("p_mesh_c.ply", p_mesh_crop)


def visDensities(mesh, densities):
    if type(densities) is not np.ndarray:
        densities = np.asarray(densities)

    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()

    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh])

# Posssion


def Posssion(pcd, vis_densities=False):
    pcd_arr = np.asarray(pcd.points)

    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=10)

    densities = np.asarray(densities)
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    poisson_mesh.remove_vertices_by_mask(vertices_to_remove)

    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)
    o3d.visualization.draw_geometries([p_mesh_crop])

    # 可视化3d形状的密度，紫色表明低密度，黄色表明高密度
    if vis_densities:
        visDensities(p_mesh_crop, densities)

    triangles = np.asarray(p_mesh_crop.triangles)
    vertices = np.asarray(p_mesh_crop.vertices)

    remove_index = []
    for i in range(triangles.shape[0]):
        p0 = vertices[triangles[i][0]]
        p1 = vertices[triangles[i][1]]
        p2 = vertices[triangles[i][2]]

        area = ComputeTriangleArea(p0, p1, p2)
        if area > 0.000025:
            remove_index.append(i)
        elif  np.linalg.norm(p1 - p0) >= 0.01 or np.linalg.norm(p2 - p0) >= 0.01 or np.linalg.norm(p2 - p1) >= 0.01:
            remove_index.append(i)

    print("before triangles shape: ", triangles.shape)
    p_mesh_crop.remove_triangles_by_index(remove_index)
    p_mesh_crop.remove_unreferenced_vertices()

    # triangles = np.asarray(p_mesh_crop.triangles)
    # print("after triagnles shape: " , triangles.shape)

    # # LOD
    # lods_num = [triangles.shape[0] , int(triangles.shape[0] * 0.1) , int(triangles.shape[0] * 0.01)]
    # lods = lod_mesh_export(p_mesh_crop, lods_num, ".ply", os.getcwd())
    # o3d.visualization.draw_geometries([lods[lods_num[2]]])
    o3d.io.write_triangle_mesh("p_mesh_c.ply", p_mesh_crop)


def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods = {}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        save_path = os.path.join(path, "lod_" + str(i) + extension)
        print(save_path)
        o3d.io.write_triangle_mesh(save_path, mesh_lod)
        mesh_lods[i] = mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i", "--input", help="input pointcloud file", required=True)
    args = parser.parse_args()
    return vars(args)


def parsePointcloudFormat(file_name):
    pcd = None
    if file_name[-3:] == "xyz":
        point_cloud = np.loadtxt(file_name, skiprows=1)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:6]/255)
    elif file_name[-3:] == "ply":
        pcd = o3d.io.read_point_cloud(file_name)

    return pcd


def main():
    args = parse_args()

    # point_cloud = np.loadtxt(args["input"], skiprows=1)

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
    # pcd.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:6]/255)

    # # pcd = o3d.io.read_point_cloud(args["input"])

    pcd = parsePointcloudFormat(args["input"])

    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(10)
    # o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    Posssion(pcd, vis_densities=True)


if __name__ == '__main__':
    main()
