/************************************************************************************************
@filename    :main.cpp
@brief       :c++版本possion重建
@time        :2021/06/22 16:20:43
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

typedef open3d::geometry::PointCloud PointCloud;

void visDensities(const std::shared_ptr<open3d::geometry::TriangleMesh> mesh, const std::vector<double> &densities)
{
}

inline double computeDist(const Eigen::Vector3d &pt1 , const Eigen::Vector3d &pt2)
{
    double diff_x = pt2[0] - pt1[0];
    double diff_y = pt2[1] - pt1[1];
    double diff_z = pt2[2] - pt1[2];

    return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

void filterMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
{
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;
    std::vector<Eigen::Vector3d> vertices=  mesh->vertices_;
    size_t triangle_num = triangles.size();
   
    std::vector<size_t> remove_index;
    for(size_t i = 0 ; i < triangle_num ; ++i)
    {
        auto p0 = vertices[triangles[i][0]];
        auto p1 = vertices[triangles[i][1]];
        auto p2 = vertices[triangles[i][2]];

        auto area = mesh->GetTriangleArea(i);
        if (area > 0.000025)
        {
            remove_index.push_back(i);
        }
        else if(computeDist(p0 , p1) >= 0.01 || computeDist(p1 , p2) >= 0.01 || computeDist(p0 , p2) >= 0.01)
        {
            remove_index.push_back(i);
        }
    }

    mesh->RemoveTrianglesByIndex(remove_index);
    mesh->RemoveUnreferencedVertices();
}

bool Possion(std::shared_ptr<PointCloud> pcd, bool vis_density = false)
{
    auto res = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pcd, 10);
    auto possion_mesh = std::get<0>(res);
    auto densities = std::get<1>(res);
    size_t point_number = densities.size();

    std::vector<bool> vertices_to_remove(point_number, false);
    for (int i = 0; i < point_number; ++i)
    {
        if (densities[i] <= 0.05)
        {
            vertices_to_remove[i] = true;
        }
    }

    possion_mesh->RemoveVerticesByMask(vertices_to_remove);
    if (vis_density)
    {
        visDensities(possion_mesh, densities);
    }

    filterMesh(possion_mesh);
    open3d::io::WriteTriangleMeshToPLY("p_mesh.ply", *possion_mesh, true, true, true, true, false, true);

    return true;
}

int main(int argc, char **argv)
{
    auto pcd = open3d::io::CreatePointCloudFromFile(argv[1]);
    pcd->EstimateNormals();
    pcd->OrientNormalsConsistentTangentPlane(10);
    Possion(pcd, false);

    return 0;
}
