#pragma once

#include <basic.hpp>
#include <random>
#include <string>
#include <yaml-cpp/yaml.h>
#include <gazebo/common/common.hh>

#include <sensor_msgs/Imu.h>

#include <rosbag/bag.h>

namespace imu
{
using IV3i = ignition::math::Vector3i;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

#define INIT_IMU_CLASS          \
int imu::Imu::imu_cnt;          \
int imu::Imu::imu_hz;           \
ignition::math::Vector3d imu::Imu::grav;            \

class Imu
{
protected:
    /* data */
    IP6d Twb, Tbi;
    IV3d vel, acc, omg;
    vec_t<float> ba, bg, nang;

    double dt, cof;

    std::string imu_name;

    sensor_msgs::Imu msg;

    GenNoise gn;

public:
    static int imu_cnt;
    static int imu_hz;
    static IV3d grav;
    
    Imu(YAML::Node& yml)
    {
        imu_name = std::string("imu") + std::to_string(imu_cnt);

        vec_t<float> tbc_vec = yml["ext"].as<vec_t<float>>();
        IV3d pos(tbc_vec[4], tbc_vec[5], tbc_vec[6]);
        IQd rot(tbc_vec[3], tbc_vec[0], tbc_vec[1], tbc_vec[2]);
        Tbi = IP6d(pos, rot);

        ba = yml["ba"].as<vec_t<float>>();
        bg = yml["bg"].as<vec_t<float>>();
        nang = yml["nang"].as<vec_t<float>>();

        msg.header.frame_id = imu_name;

        dt = 1.0 / imu_hz;
        cof = 1.0 / std::sqrt(dt);

        imu_cnt++;
    }

    std::string GetImuName() const { return imu_name; }

    void FromBaseKin(const IP6d& Twb_, const IV3d& v, const IV3d& dv, const IV3d& w, const IV3d& dw)
    {
        Twb = Twb_;

        // now the vel_ is the body vel in world frame, we need to 
        // get vel of imu in world frame but expressed in imu frame
        auto Rbw = Twb.Rot().Inverse();
        auto Rib = Tbi.Rot().Inverse();

        auto vb = Rbw * v;
        auto dvb = Rbw * (dv + grav);
        auto omg_b = Rbw * w;
        auto domg_b = Rbw * dw;
        auto vbi = omg_b.Cross(Tbi.Pos());

        vel = Rib * (vb + vbi);
        omg = Rib * omg_b;
        acc = Rib * (dvb + omg_b.Cross(omg_b.Cross(vbi)) + domg_b.Cross(Tbi.Pos()));
    }

    void WriteToBag(rosbag::Bag& bag, double t, std::string topic)
    {
        msg.header.stamp = ros::Time(t);

        msg.angular_velocity.x = omg.X() + bg[0] + nang[1] * cof * gn.getNoise(0, 1.0);
        msg.angular_velocity.y = omg.Y() + bg[1] + nang[1] * cof * gn.getNoise(0, 1.0);
        msg.angular_velocity.z = omg.Z() + bg[2] + nang[1] * cof * gn.getNoise(0, 1.0);

        msg.linear_acceleration.x = acc.X() + ba[0] + nang[0] * cof * gn.getNoise(0, 1.0);
        msg.linear_acceleration.y = acc.Y() + ba[1] + nang[0] * cof * gn.getNoise(0, 1.0);
        msg.linear_acceleration.z = acc.Z() + ba[2] + nang[0] * cof * gn.getNoise(0, 1.0);
        bag.write(topic, msg.header.stamp, msg);
    }

};


} // namespace imu
