#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using Pose6 = Eigen::Isometry3d;
using V3 = Eigen::Vector3d;
using Qd = Eigen::Quaterniond;
using plane_t = Eigen::Vector4d;

using pt_t = pcl::PointXYZI;
using pn_t = pcl::PointXYZINormal;
using pc_t = pcl::PointCloud<pt_t>;
using pc_ptr = pc_t::Ptr;

template <typename T>
using pc_t_ = pcl::PointCloud<T>;

template <typename T>
using pc_ptr_ = typename pc_t_<T>::Ptr;

template <typename T>
using vec_t = std::vector<T>;

template <typename T>
using vvec_t = std::vector<std::vector<T>>;

struct Const
{
    static constexpr double to_rad = M_PI / 180.0; 
    static constexpr double to_deg = 180.0 / M_PI;
};

class GenNoise
{
protected:
    
    std::random_device rd{};
    std::mt19937 gen{rd()};

public:

    double getNoise(double std)
    {
        std::normal_distribution<double> d(0.0, std);
        return d(gen);
    }
};