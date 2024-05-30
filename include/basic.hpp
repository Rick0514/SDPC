#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <random>

using Pose6 = Eigen::Isometry3d;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using Qd = Eigen::Quaterniond;
using plane_t = Eigen::Vector4d;

using pt_t = pcl::PointXYZI;
using pn_t = pcl::PointXYZINormal;
using pc_t = pcl::PointCloud<pt_t>;
using pc_ptr = pc_t::Ptr;

using prgb_t = pcl::PointXYZRGB;
using pcrgb_t = pcl::PointCloud<prgb_t>;

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
    
    // https://stackoverflow.com/questions/71624001/deleted-function-error-after-using-random-lib-in-struct
    std::mt19937 gen;

public:

    GenNoise(){
        gen = std::mt19937(std::random_device{}());
    }

    double getNoise(double std)
    {
        std::normal_distribution<double> d(0.0, std);
        return d(gen);
    }

    double getNoise(double mu, double std)
    {
        std::normal_distribution<double> d(mu, std);
        return d(gen);
    }
};
