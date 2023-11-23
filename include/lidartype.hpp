#pragma once

#include <vector>
#include <sdf/sdf.hh>

#include <ignition/math.hh>
#include <rosbag/bag.h>

#include <pctypes.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

namespace lidartype
{

static const double Deg2Rad = M_PI / 180;

using std::vector;
using std::pair;
using point_t = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;

struct ig_pc
{
    double timestamp;
    vector<point_t> pc;
    vector<int> ring;
};

using v_time_pc = vector<ig_pc>; // vector of a bunch of points with same timestamp

class LidarBase
{
protected:
    
    int hz;
    double max_range;
    double min_range;

public:

    LidarBase(sdf::ElementPtr sdf)
    {
        auto range_sdf = sdf->GetElement("ray")->GetElement("range");
        min_range = range_sdf->Get<double>("min");
        max_range = range_sdf->Get<double>("max");

        hz = sdf->Get<int>("hz");
        gzmsg << "hz: " << hz << std::endl;
    }

    int getHz() const { return hz;  }

    virtual v_time_pc getFrame(double start_time) = 0;
    virtual void writeToBag(rosbag::Bag& bag, const v_time_pc& vp, const std::string& topic) {}

    std::string name;
};


class Velodyne : public LidarBase
{
private:
    
    int ver_n, hor_n;
    pair<double, double> ver_ang, hor_ang;

    v_time_pc end_points;
    
public:
    Velodyne(sdf::ElementPtr sdf) : LidarBase(sdf)
    {
        this->name = "velodyne";

        auto scan_sdf = sdf->GetElement("ray")->GetElement("scan");
        auto hor_sdf = scan_sdf->GetElement("horizontal");
        auto ver_sdf = scan_sdf->GetElement("vertical");
        
        ver_n = ver_sdf->Get<int>("samples");
        hor_n = hor_sdf->Get<int>("samples");

        ver_ang.first = ver_sdf->Get<double>("min_angle");
        ver_ang.second = ver_sdf->Get<double>("max_angle");

        hor_ang.first = hor_sdf->Get<double>("min_angle");
        hor_ang.second = hor_sdf->Get<double>("max_angle");

        vector<double> zen;
        zen.resize(ver_n);

        for(int i=0; i<ver_n; i++){
            zen[i] = ver_ang.first + (ver_ang.second - ver_ang.first) * i / ver_n;
            zen[i] *= Deg2Rad;
        }

        end_points.resize(hor_n);
        for(int i=0; i<hor_n; i++){
            double deg = hor_ang.first + (hor_ang.second - hor_ang.first) * i / hor_n * Deg2Rad;
            double t0 = 1.0 / hz * i / hor_n;
            
            IQd ray;
            ig_pc ver_pc;
            ver_pc.timestamp = t0;
            ver_pc.pc.resize(ver_n);
            ver_pc.ring.resize(ver_n);
            for(int j=0; j<ver_n; j++){
                ray.Euler(point_t(0.0, zen[j], deg));
                ver_pc.pc[j] = ray * point_t(max_range, 0, 0);
                ver_pc.ring[j] = j;
            }
            end_points[i] = ver_pc;
        }
    }

    virtual v_time_pc getFrame(double start_time) override
    {
        auto eps = end_points;
        for(auto& e : eps)   e.timestamp += start_time;
        return eps;
    }

    virtual void writeToBag(rosbag::Bag& rbag, const v_time_pc& vp, const std::string& topic) override
    {
        if(vp.empty())  return;

        pcl::VelodynePC vpc;
        vpc.reserve(ver_n * hor_n);
        
        for(const auto& ps : vp){
            for(int i=0; i<ps.pc.size(); i++){
                pcl::VelodynePoint pt;
                pt.time = ps.timestamp;
                pt.ring = ps.ring[i];
                pt.intensity = 100.0;
                pt.x = ps.pc[i].X();
                pt.y = ps.pc[i].Y();
                pt.z = ps.pc[i].Z();
                vpc.emplace_back(std::move(pt));
            }
        }

        sensor_msgs::PointCloud2 ros_pc;
        pcl::toROSMsg(vpc, ros_pc);
        ros_pc.header.stamp = ros::Time().fromSec(vp.front().timestamp);
        ros_pc.header.frame_id = name;
        rbag.write(topic, ros_pc.header.stamp, ros_pc);
    }
};

// class Avia : public LidarBase
// {

// };

} // namespace lidartype


