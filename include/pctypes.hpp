#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

namespace pcl{
struct LivoxPointXyzrtl{
    PCL_ADD_POINT4D;
    float reflectivity; /**< Reflectivity   */
    uint8_t tag;        /**< Livox point tag   */
    uint8_t line;       /**< Laser line id     */
    EIGEN_ALIGN16;
};

struct EIGEN_ALIGN16 VelodynePoint {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using VelodynePC = PointCloud<VelodynePoint>;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointXyzrtl,  
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)