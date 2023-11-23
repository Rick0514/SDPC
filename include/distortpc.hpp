#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/ode/common.h>

#include <rosbag/bag.h>

#include <sixdofcubicspline.hpp>
#include <lidartype.hpp>

namespace gazebo
{

namespace physics{
    class OneRayShape;
}

using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

class DistortPC : public SensorPlugin
{
private:

    ros::NodeHandlePtr rn;
    // for vis debug
    ros::Publisher pc_pub;

    sensors::SensorPtr parentSensor;

    physics::CollisionPtr laserCollision;
    physics::WorldPtr world;
    physics::EntityPtr parentEntity;
    boost::shared_ptr<physics::OneRayShape> laserShape;

    lidartype::LidarBase* lidar;
    SixDofCubicSpline* sixsp;

    event::ConnectionPtr update_connection_;

    rosbag::Bag rbag;

    double total_time;
    string path_fn;
    string save_bag;
    string lidar_topic;
    string lidar_type;
    bool save_pcd;
    string pcd_dir;

    pcl::PointCloud<pcl::PointXYZ>::Ptr dist_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr undist_pc;

public:
    DistortPC();
    ~DistortPC();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf);
    void DistortPCHandler();
};

};