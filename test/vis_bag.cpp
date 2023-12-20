#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <thread>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

int N_SCANS = 10;
float blind = 0.5f;

PointCloudXYZI::Ptr livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {

    int plsize = msg->point_num;
    PointCloudXYZI::Ptr cloud = pcl::make_shared<PointCloudXYZI>();
    cloud->reserve(plsize);

    for (uint i = 1; i < plsize; i++) {
        if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 ||
            (msg->points[i].tag & 0x30) == 0x00)) {

            PointType p;
            p.x = msg->points[i].x;
            p.y = msg->points[i].y;
            p.z = msg->points[i].z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points
            double dist = p.x * p.x + p.y * p.y + p.z * p.z;
            if (dist < blind * blind)   continue;
            cloud->push_back(p);
        }
    }

    return cloud;
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "vis_bag");

    ros::NodeHandle rn("~");

    string bag_fn;
    rn.getParam("bag_fn", bag_fn);
    rn.getParam("N_SCANS", N_SCANS);
    cout << "bag file name: " << bag_fn << endl;
    
    string vis_topic = "vis_pc";
    string vis_pose_topic = "vis_pose";
    ros::Publisher pub_pose = rn.advertise<nav_msgs::Odometry>(vis_pose_topic, 1, true);
    ros::Publisher pub = rn.advertise<sensor_msgs::PointCloud2>(vis_topic, 1, true);

    rosbag::Bag bag;
    bag.open(bag_fn, rosbag::bagmode::Read);

    vector<string> lidar_topics{"/livox/lidar", "lidar_points"};
    deque<nav_msgs::Odometry> lidar_odom;

    for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery("lidar_pose")))
    {
        nav_msgs::OdometryConstPtr od = m.instantiate<nav_msgs::Odometry>();
        lidar_odom.push_back(*od);
    }

    cout << "lidar_odom size: " << lidar_odom.size() << endl;

    PointCloudXYZI::Ptr pc = pcl::make_shared<PointCloudXYZI>();

    for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(lidar_topics)))
    {   
        std::string tp = m.getTopic();
        auto cur_time =  m.getTime();

        if(tp == lidar_topics[0]){
            // livox
            auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
            pc = livoxHandler(msg);
        }else if(tp == lidar_topics[1]){
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*msg, *pc);
        }

        nav_msgs::Odometry od = lidar_odom.front();
        lidar_odom.pop_front();
        
        auto gp = od.pose.pose.position;
        auto gq = od.pose.pose.orientation;
        Eigen::Vector3f et(gp.x, gp.y, gp.z);
        Eigen::Quaternionf eq(gq.w, gq.x, gq.y, gq.z);
        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3, 3>(0, 0) = eq.toRotationMatrix();
        trans.block<3, 1>(0, 3) = et;
        
        pcl::transformPointCloud(*pc, *pc, trans);
        
        sensor_msgs::PointCloud2 ros_pc;
        pcl::toROSMsg(*pc, ros_pc);
        ros_pc.header.stamp = cur_time;
        ros_pc.header.frame_id = "map";

        pub.publish(ros_pc);
        pub_pose.publish(od);

        ros::spinOnce();
        std::this_thread::sleep_for(chrono::milliseconds(100));

        if(!rn.ok())    break;
    }

    cout << "exit!" << endl;

    return 0;
}

