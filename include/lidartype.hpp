#pragma once

#include <vector>
#include <sdf/sdf.hh>
#include <yaml-cpp/yaml.h>

#include <ignition/math.hh>

#include <ros/time.h>
#include <rosbag/bag.h>

#include <pctypes.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>


namespace lidartype
{

static const double Deg2Rad = M_PI / 180;

using std::vector;
using std::pair;
using point_t = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using v_time_pc = vector<ig_pc>; // vector of a bunch of points with same timestamp

struct ig_pc
{
    double timestamp;
    vector<point_t> pc;
    vector<int> ring;
};

struct LivoxRotateInfo {
    double time;
    double azimuth;
    double zenith;
    uint8_t line;
};

class LidarBase
{
protected:
    
    float hz;
    double max_range;
    double min_range;

    double noise_std;

public:

    LidarBase(YAML::Node& yml){
        auto hz = yml["hz"].as<float>();
        auto noise_std = yml["dist_noise"].as<double>();
        auto rg = yml["range"].as<vector<float>>();
        min_range = rg[0];
        max_range = rg[1];
    }

    LidarBase(sdf::ElementPtr sdf)
    {
        auto range_sdf = sdf->GetElement("ray")->GetElement("range");
        auto noise_sdf = sdf->GetElement("ray")->GetElement("noise");
        min_range = range_sdf->Get<double>("min");
        max_range = range_sdf->Get<double>("max");

        noise_std = noise_sdf->Get<double>("stddev");
        gzmsg << "noise std: " << noise_std << std::endl;        

        hz = sdf->Get<int>("hz");
        gzmsg << "hz: " << hz << std::endl;        
    }

    static bool readCsvFile(std::string file_name, std::vector<std::vector<double>>& datas) {
        std::fstream file_stream;
        file_stream.open(file_name, std::ios::in);
        if (file_stream.is_open()) {
            std::string header;
            std::getline(file_stream, header, '\n');
            while (!file_stream.eof()) {
                std::string line_str;
                std::getline(file_stream, line_str, '\n');
                std::stringstream line_stream;
                line_stream << line_str;
                std::vector<double> data;
                try {
                    while (!line_stream.eof()) {
                        std::string value;
                        std::getline(line_stream, value, ',');
                        data.push_back(std::stod(value));
                    }
                } catch (...) {
                    std::cerr << "cannot convert str:" << line_str << "\n";
                    continue;
                }
                datas.push_back(data);
            }
            std::cerr << "data size:" << datas.size() << "\n";
            return true;
        } else {
            std::cerr << "cannot read csv file!" << file_name << "\n";
        }
        return false;
    }

    static void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<LivoxRotateInfo> &avia_infos) {
        avia_infos.reserve(datas.size());
        for (int i = 0; i < datas.size(); ++i) {
            auto &data = datas[i];
            if (data.size() == 3) {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * Deg2Rad;
                avia_infos.back().zenith = data[2] * Deg2Rad - M_PI_2;
                avia_infos.back().line = i % 6;
            } else {
                gzwarn << "convert to rotate info err: data size is not 3!" << std::endl;
            }
        }
    }

    float getHz() const { return hz;  }
    double getNoiseStd() const { return noise_std; }
    double getMaxRange() const { return max_range; }

    virtual v_time_pc getFrame(double start_time) = 0;
    virtual void writeToBag(rosbag::Bag& bag, const v_time_pc& vp, const std::string& topic) {}

    std::string name;

    static std::shared_ptr<LidarBase> create(string type, YAML::Node& yml)
    {
        if(type == "velo" || type == "velodyne"){
            return std::make_shared<Velodyne>(yml);
        }else if(type == "avia"){
            return std::make_shared<Avia>(yml);
        }
    }
};


class Velodyne : public LidarBase
{
private:
    
    int ver_n, hor_n;
    pair<double, double> ver_ang, hor_ang;

    v_time_pc end_points;
    
public:

    Velodyne(YAML::Node& yml) : LidarBase(yml)
    {
        this->name = "velo";
        
        auto yyml = yml[name];
        auto ver_ = yyml["ver_range"].as<vector<double>>();
        auto hor_ = yyml["hor_range"].as<vector<double>>();
        ver_ang.first = ver_[0];
        ver_ang.second = ver_[1];
        hor_ang.first = hor_[0];
        hor_ang.second = hor_[1];

        ver_n = yyml["ver_sample"].as<int>();
        hor_n = yyml["hor_sample"].as<int>();

        ig_pc igpc;
        igpc.pc.resize(hor_n * ver_n);
        igpc.ring.resize(hor_n * ver_n);

        vector<double> zen;
        zen.resize(ver_n);
        for(int i=0; i<ver_n; i++){
            zen[i] = ver_ang.first + (ver_ang.second - ver_ang.first) * i / ver_n;
            zen[i] *= Deg2Rad;
        }

        for(int i=0; i<hor_n; i++){
            double deg = hor_ang.first + (hor_ang.second - hor_ang.first) * i / hor_n * Deg2Rad;
            
            IQd ray;
            for(int j=0; j<ver_n; j++){
                ray.Euler(point_t(0.0, zen[j], deg));
                igpc.pc[i * ver_n + j] = ray * point_t(max_range, 0, 0);
                igpc.ring[i * ver_n + j] = j;
            }
        }
        
        end_points.push_back(igpc);
    }

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
        double f_stp = vp.front().timestamp;

        for(const auto& ps : vp){
            for(int i=0; i<ps.pc.size(); i++){
                pcl::VelodynePoint pt;
                pt.time = ps.timestamp - f_stp;
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
        ros_pc.header.stamp = ros::Time().fromSec(f_stp);
        ros_pc.header.frame_id = name;
        rbag.write(topic, ros_pc.header.stamp, ros_pc);
    }
};

class Avia : public LidarBase
{

    int samples;
    int downsample{1};
    std::vector<LivoxRotateInfo> aviainfos;

    int start_idx{0};
    string scan_dir;

public:

    Avia(YAML::Node& yml) : LidarBase(yml)
    {
        this->name = "avia";

        auto yyml = yml["livox"];
        scan_dir = yml["scan_dir"].as<string>();
        samples = yml["samples"].as<int>();

        vector<vector<double>> datas;
        readCsvFile(scan_dir + this->name + ".csv", datas);
        convertDataToRotateInfo(datas, aviainfos);
    }

    Avia(sdf::ElementPtr sdf, string scan_dir_) : LidarBase(sdf), scan_dir(scan_dir_)
    {
        this->name = "avia";

        auto scan_sdf = sdf->GetElement("ray")->GetElement("scan");
        auto hor_sdf = scan_sdf->GetElement("horizontal");
        
        samples = hor_sdf->Get<int>("samples");
        downsample = sdf->Get<int>("downsample");

        vector<vector<double>> datas;
        readCsvFile(scan_dir + this->name + ".csv", datas);
        convertDataToRotateInfo(datas, aviainfos);
    }

    virtual v_time_pc getFrame(double start_time) override
    {
        v_time_pc frame(samples);
        double ofs_time;

        for(int i=0; i<samples; i++){
            int idx = (start_idx + i * downsample) % aviainfos.size();
            const LivoxRotateInfo& info = aviainfos[idx];

            if(i == 0)  ofs_time = info.time;

            IQd ray;
            ig_pc pc;
        
            ray.Euler(point_t(0.0, info.zenith, info.azimuth));
            auto axis = ray * point_t(max_range, 0.0, 0.0);
            pc.timestamp = start_time + (info.time - ofs_time);
            pc.pc.push_back(axis);
            pc.ring.push_back(info.line);   // use ring to store line

            frame[i] = pc;
        }

        start_idx += samples * downsample;
        start_idx %= aviainfos.size();

        return frame;            
    }

    virtual void writeToBag(rosbag::Bag& rbag, const v_time_pc& vp, const std::string& topic) override
    {
        if(vp.empty())  return;
                
        livox_ros_driver::CustomMsg msg;
        msg.header.frame_id = name;

        ros::Time rt(vp.front().timestamp);
        msg.timebase = rt.toNSec();
        msg.header.stamp = rt;
        msg.point_num = vp.size();
        msg.points.reserve(msg.point_num);

        for(const auto& p : vp){
            
            const auto& pp = p.pc[0];            
            livox_ros_driver::CustomPoint cp;

            cp.x = pp.X();
            cp.y = pp.Y();
            cp.z = pp.Z();

            cp.reflectivity = 100;
            cp.tag = 0x10;
            cp.line = p.ring[0];
            cp.offset_time = p.timestamp * 1e9 - msg.timebase;

            msg.points.push_back(cp);
        }

        rbag.write(topic, msg.header.stamp, msg);
    }

};

} // namespace lidartype



