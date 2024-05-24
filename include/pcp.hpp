#pragma once

#include "basic.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>

// filter out nan and too distant points
template <typename T>
void toValidPC(pc_ptr_<T>& pc, float range)
{
    pc_t_<T> tmp;
    tmp.reserve(pc->size());
    for(const auto& pt : pc->points){
        if(fabs(pt.x) < range && fabs(pt.y) < range && fabs(pt.z) < range)
            tmp.push_back(pt);
    }
    pc->swap(tmp);
}

template <typename T>
void savePCD(const std::string& path, const pc_t_<T>& pc)
{
    pcl::io::savePCDFileBinary(path, pc);
}

class VoxelDownSample
{
protected:

    template <typename T>
    using PC = pcl::PointCloud<T>;

    float _grid_size;
    float _inverse_grid_size;

    Eigen::Array3f _max_p, _min_p;

public:
    VoxelDownSample() = default;
    VoxelDownSample(float gs)
    {
        setGridSize(gs);
    }

    void setGridSize(float sz){
        _grid_size = sz;
        _inverse_grid_size = 1.0f / _grid_size;
    }

    template <typename PointType>
    void filter(const typename PC<PointType>::ConstPtr& cloud, typename PC<PointType>::Ptr& out)
    {
        if(cloud->empty())  return;

        out->header = cloud->header;
        out->sensor_origin_ = cloud->sensor_origin_;
        out->sensor_orientation_ = cloud->sensor_orientation_;

        _max_p = cloud->points[0].getArray3fMap();
        _min_p = _max_p;

        // find max and min first
        for(const auto& p : cloud->points){
            pcl::Array3fMapConst pt = p.getArray3fMap();
            _max_p = _max_p.max(pt);
            _min_p = _min_p.min(pt);
        }

        // printf("inverse gs: %f\n", _inverse_grid_size);

        Eigen::Array3i max_b, min_b;
        min_b[0] = static_cast<int> (std::floor (_min_p[0] * _inverse_grid_size));
        max_b[0] = static_cast<int> (std::floor (_max_p[0] * _inverse_grid_size));
        min_b[1] = static_cast<int> (std::floor (_min_p[1] * _inverse_grid_size));
        max_b[1] = static_cast<int> (std::floor (_max_p[1] * _inverse_grid_size));
        min_b[2] = static_cast<int> (std::floor (_min_p[2] * _inverse_grid_size));
        max_b[2] = static_cast<int> (std::floor (_max_p[2] * _inverse_grid_size));

        // printf("minb: %d, %d, %d\n", min_b[0], min_b[1], min_b[2]);
        // printf("maxb: %d, %d, %d\n", max_b[0], max_b[1], max_b[2]);

        int dx = max_b[0] - min_b[0] + 1;
        int dy = max_b[1] - min_b[1] + 1;

        std::unordered_map<size_t, std::vector<size_t>> _map;

        for(size_t i=0; i<cloud->size(); i++){
            const auto& pt = cloud->points[i];
            size_t ijk0 = static_cast<size_t> (std::floor (pt.x * _inverse_grid_size) - static_cast<float> (min_b[0]));
            size_t ijk1 = static_cast<size_t> (std::floor (pt.y * _inverse_grid_size) - static_cast<float> (min_b[1]));
            size_t ijk2 = static_cast<size_t> (std::floor (pt.z * _inverse_grid_size) - static_cast<float> (min_b[2]));

            size_t idx = ijk0 + ijk1 * dx + ijk2 * dx * dy;

            if(_map.count(idx) == 0){
                _map.emplace(idx, std::vector<size_t>{i});
            }else{
                _map[idx].emplace_back(i);
            }
        }
        
        PC<PointType>* out_ = out.get();
        PC<PointType> out_tmp;

        if(cloud.get() == out.get())
            out_ = &out_tmp;

        out_->clear();
        out_->reserve(_map.size());

        for(const auto& [_, vb] : _map){
            pcl::CentroidPoint<PointType> centroid;

            // fill in the accumulator with leaf points
            for (auto&& idx : vb)
                centroid.add(cloud->points[idx]);  
            
            PointType pt;
            centroid.get(pt);
            // PointType pt = cloud->points[vb.front()];
            out_->emplace_back(pt);
        }

        if(cloud.get() == out.get())    pcl::copyPointCloud(out_tmp, *out);
    }

};
