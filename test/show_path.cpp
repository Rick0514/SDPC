#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <sixdofcubicspline.hpp>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <icecream.hpp>

using namespace std;

using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

int main(int argc, char *argv[])
{

    // auto data = parsePath("/home/rick/chore/lips/lips_matlab/input/path_test.txt");

    // int hz = 10;
    // double total_time = 10.0;

    // vector<double> t, x;
    // int n = data.size();

    // for(auto& e : data){
    //     t.push_back(e[0]);
    //     x.push_back(e[1]);
    //     cout << e[0] << " " << e[1] << endl;
    // }

    // double time_scale = *std::max_element(t.begin(), t.end());
    // for(auto& e : t)   e = (e / time_scale) * total_time;
    
    // tk::spline s;
    // s.set_boundary(tk::spline::bd_type::first_deriv, 0,
    //     tk::spline::bd_type::first_deriv, 0);
    // s.set_points(t, x);

    // cout << s(2.5) << endl;

    // get rospackage dir
    string pkg_dir = ros::package::getPath("sdpc");
    // load params from mld.yaml
    YAML::Node yml = YAML::LoadFile(pkg_dir + "/config/mld.yaml");

    ros::init(argc, argv, "show_path_node");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/path", 1, true);

    double total_time, dt;
    string path_fn;
    // nh.getParam("total_time", total_time);
    // nh.getParam("dt", dt);
    // nh.getParam("path_fn", path_fn);

    // load params from yml
    total_time = yml["path"]["total_time"].as<double>();
    dt = yml["path"]["dt"].as<double>();
    path_fn = pkg_dir + yml["path"]["path_fn"].as<string>(); 
    
    IC(total_time, dt, path_fn);

    SixDofCubicSpline scs(total_time, path_fn);
    double start_time = 0;

    nav_msgs::Path pf;
    pf.header.frame_id = "map";
    pf.header.stamp = ros::Time::now();

    while(start_time < total_time){

        auto pose = scs.getPose(start_time);
        
        geometry_msgs::PoseStamped pst;
        pst.header = pf.header; 
        geometry_msgs::Pose ps;
        ps.position.x = pose.Pos().X();
        ps.position.y = pose.Pos().Y();
        ps.position.z = pose.Pos().Z();
        ps.orientation.x = pose.Rot().X();
        ps.orientation.y = pose.Rot().Y();
        ps.orientation.z = pose.Rot().Z();
        ps.orientation.w = pose.Rot().W();

        pst.pose = ps;
        pf.poses.push_back(pst);

        start_time += dt;
    }

    while(ros::ok()){
        pub.publish(pf);
        ros::spinOnce();
    }

    return 0;
}
