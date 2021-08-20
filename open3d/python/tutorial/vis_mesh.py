import open3d as o3d
mesh = o3d.io.read_triangle_mesh("/home/han/Desktop/7-16-result/livox/mesh.ply")
print(mesh)
# mesh.compute_vertex_normals()
# mesh.paint_uniform_color([1, 0.7, 0])
o3d.visualization.draw_geometries([mesh])
