#ifndef _HULL_SHRINKER_HPP
#define _HULL_SHRINKER_HPP

#include <vector>
#include <Eigen/Dense>

#include "quickhull.hpp"
#include "geo_utils.hpp"

namespace HullShrinker{

typedef float  float32_t;
typedef double float64_t;

// point-normal representation
struct Plane{
    Eigen::Vector3d _pt;
    Eigen::Vector3d _normal;
    Plane(const Eigen::Vector3d& pt, const Eigen::Vector3d& normal):_pt(pt), _normal(normal){};
    Plane(){};
    ~Plane(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// triangle mesh
struct Mesh {
    Eigen::Vector3d _a;
    Eigen::Vector3d _b;
    Eigen::Vector3d _c;
    Eigen::Vector3d _normal;

    Mesh(){};
    ~Mesh(){};
    Mesh(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, 
         const Eigen::Vector3d& normal): _a(a), _b(b), _c(c), _normal(normal){};
    Mesh(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
         : _a(a), _b(b), _c(c){calcNormal();};
    void calcNormal() 
    { 
        Eigen::Vector3d ca = _a - _c;
        Eigen::Vector3d cb = _b - _c;
        _normal = ca.cross(cb);
        _normal.normalize();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class HullShrinker{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<HullShrinker> Ptr;
private:
    std::vector<quickhull::Vector3<float64_t>> _ori_vertices;
    const double _qh_eps = std::min(1e-6, quickhull::defaultEps<double>());

public:
    HullShrinker()=delete;
    HullShrinker(const std::vector<Eigen::Vector3d>& ori_vertices_eigen);
    HullShrinker(const std::vector<quickhull::Vector3<float64_t>>& ori_vertices_vec);
    HullShrinker(const quickhull::VertexDataSource<float64_t>& ori_vertices_buffer);
    ~HullShrinker(){};
    void shrinkToConvexHull(quickhull::ConvexHull<float64_t>& final_hull);
    quickhull::ConvexHull<float64_t> shrinkToConvexHull();

private:
    int8_t isInsideHull(const std::vector<Mesh*>& meshes, 
                        const Eigen::Vector3d& pt, float64_t& min_distance, int32_t& min_idx);
    float64_t disPt2Plane(const Eigen::Vector3d& pt, 
                        const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_pt);
    bool isSamePos(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const float64_t& eps=1e-2)
    {
        for (int32_t i=0; i<a.rows(); ++i)
            if (std::fabs(a(i) - b(i)) > eps)
                return false;
        return true;
    }
};

inline
HullShrinker::HullShrinker(const std::vector<Eigen::Vector3d>& ori_vertices_eigen)
{
    _ori_vertices.resize(ori_vertices_eigen.size());
    for (uint64_t i=0; i<_ori_vertices.size(); ++i)
        _ori_vertices[i] = quickhull::Vector3<float64_t>(ori_vertices_eigen[i](0),
                                                         ori_vertices_eigen[i](1),
                                                         ori_vertices_eigen[i](2));
    
    return;
}

inline
HullShrinker::HullShrinker(const std::vector<quickhull::Vector3<float64_t>>& ori_vertices_vec)
{
    _ori_vertices.resize(ori_vertices_vec.size());
    for (uint64_t i=0; i<_ori_vertices.size(); ++i)
        _ori_vertices[i] = ori_vertices_vec[i];
    return;
}

inline
HullShrinker::HullShrinker(const quickhull::VertexDataSource<float64_t>& ori_vertices_buffer)
{
    _ori_vertices.resize(ori_vertices_buffer.size());
    for (uint64_t i=0; i<_ori_vertices.size(); ++i)
        _ori_vertices[i] = quickhull::Vector3<float64_t>(ori_vertices_buffer[i].x, 
                                                         ori_vertices_buffer[i].y, 
                                                         ori_vertices_buffer[i].z);

    return;
}

inline
void HullShrinker::shrinkToConvexHull(quickhull::ConvexHull<float64_t>& final_hull)
{
    final_hull = shrinkToConvexHull();
}

inline
quickhull::ConvexHull<float64_t> HullShrinker::shrinkToConvexHull()
{
    quickhull::ConvexHull<float64_t> final_hull;
    if (_ori_vertices.size() < 4)
        return final_hull;

    quickhull::QuickHull<float64_t> qh;
    auto ver_hull = qh.getConvexHull(_ori_vertices, true, false, _qh_eps);

    auto ver_vtx_buffer = ver_hull.getVertexBuffer();
    auto ver_idx_buffer = ver_hull.getIndexBuffer();
    std::vector<Mesh*> ver_meshes(ver_idx_buffer.size()/3);

    for (uint64_t i=0; i< ver_idx_buffer.size()/3; ++i)
    {
        std::vector<Eigen::Vector3d> vertices(3);
        for (int32_t j=0; j<3; ++j)
        {
            Eigen::Vector3d vertex;
            vertex << ver_vtx_buffer[ver_idx_buffer[i*3+(uint64_t)j]].x,
                      ver_vtx_buffer[ver_idx_buffer[i*3+(uint64_t)j]].y,
                      ver_vtx_buffer[ver_idx_buffer[i*3+(uint64_t)j]].z;
            vertices[j] = vertex;
        }
        Mesh* mesh = new Mesh(vertices[0], vertices[1], vertices[2]);
        ver_meshes[i] = mesh;
    }

    std::vector<float64_t> distances(ver_meshes.size(), -1.0);
    std::vector<Plane*> planes(ver_meshes.size());

    for (uint64_t i=0; i<_ori_vertices.size(); ++i)
    {
        Eigen::Vector3d pt = Eigen::Vector3d(_ori_vertices[i].x,
                                             _ori_vertices[i].y,
                                             _ori_vertices[i].z);
        float64_t min_distance = __DBL_MAX__;
        int32_t mesh_idx = -1;
        if (isInsideHull(ver_meshes, pt, min_distance, mesh_idx))
            if (min_distance > distances[mesh_idx])
            {
                distances[mesh_idx] = min_distance;
                planes[mesh_idx] = new Plane(pt, ver_meshes[mesh_idx]->_normal);
            }
    }

    Eigen::MatrixXd h_poly(planes.size(), 4);
    for (uint64_t i=0; i<planes.size(); ++i)
    {
        float64_t d = -1;
        Eigen::Vector3d normal = Eigen::Vector3d::Ones();
        if (distances[i] > 0)
        {
            d = planes[i]->_pt.transpose() * planes[i]->_normal;
            normal = planes[i]->_normal;
        }
        else
        {
            d = ver_meshes[i]->_c.transpose() * ver_meshes[i]->_normal;
            normal = ver_meshes[i]->_normal;
        }
        h_poly.block(i, 0, 1, 3) = normal.transpose();
        h_poly(i, 3) = d;
    }
    Eigen::Matrix<float64_t, 3, -1, Eigen::ColMajor> v_poly;
    geo_utils::enumerateVs(h_poly, v_poly);

    // downsample
    for (uint64_t i=0; i<v_poly.cols(); ++i)
        for (uint64_t j=i+1; j<v_poly.cols(); ++j)
            if (isSamePos(v_poly.col(i), v_poly.col(j), 1e-1))
                v_poly.col(j) = v_poly.col(i);

    final_hull = qh.getConvexHull(v_poly.data(), v_poly.cols(), true, false, _qh_eps);

    return final_hull;
}

inline
int8_t HullShrinker::isInsideHull(const std::vector<Mesh*>& meshes, const Eigen::Vector3d& pt, float64_t& min_distance, int32_t& min_idx)
{
    min_idx = -1;
    min_distance = __DBL_MAX__;

    for (uint64_t i=0; i<meshes.size(); ++i)
    {
        auto dis = disPt2Plane(pt, meshes[i]->_normal, meshes[i]->_c);
        if (dis<1e-3) return false;
        else if (dis<min_distance)
        {
            min_distance = dis;
            min_idx = (int32_t)i;
        }
    }
    return true;
}

inline
float64_t HullShrinker::disPt2Plane(const Eigen::Vector3d& pt, const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_pt)
{
    return ((pt - plane_pt).transpose()*plane_normal);
}
};

#endif