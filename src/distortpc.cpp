
#include <chrono>
#include <onerayshape.hpp>
#include "gazebo/physics/physics.hh"
#include <distortpc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace gazebo
{
using std::endl;

GZ_REGISTER_SENSOR_PLUGIN(DistortPC)

DistortPC::DistortPC(){}

DistortPC::~DistortPC()
{
    parentSensor.reset();
    world.reset();
    laserShape.reset();
}

void DistortPC::Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf)
{
    gzmsg << "Load distort lidar plugin now!!" << endl;
    rn = boost::make_shared<ros::NodeHandle>();

    rn->getParam("total_time", total_time);
    rn->getParam("path_fn", path_fn);
    rn->getParam("lidar_topic", lidar_topic);
    rn->getParam("save_bag", save_bag);
    rn->getParam("lidar_type", lidar_type);
    rn->getParam("save_pcd", save_pcd);
    rn->getParam("pcd_dir", pcd_dir);
    gzmsg << "path_fn: " << path_fn << endl;
    gzmsg << "save_pcd: " << save_pcd << endl;
    gzmsg << "pcd_dir: " << pcd_dir << endl;

    pc_pub = rn->advertise<sensor_msgs::PointCloud2>(lidar_topic, 5);
    sixsp = new SixDofCubicSpline(total_time, path_fn);

    if(lidar_type == "velodyne"){
        lidar = new lidartype::Velodyne(sdf);
    }else{
        gzerr << "No [" << lidar_type << "] such lidar type!!" << endl;
        throw std::invalid_argument("");
    }

    rbag.open(save_bag, rosbag::bagmode::Write);

    dist_pc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    undist_pc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    parentSensor = _parent;
    world = physics::get_world(parentSensor->WorldName());
    parentEntity = world->EntityByName(parentSensor->ParentName());

    // init engine
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("ray", parentSensor->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(parentSensor->Pose());
    laserCollision->SetInitialRelativePose(parentSensor->Pose());

    // init shape
    laserShape.reset(new physics::OneRayShape(laserCollision));
    laserShape->Load(sdf);
    laserShape->Init();
    laserCollision->SetShape(laserShape);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DistortPC::DistortPCHandler, this));

}

void DistortPC::DistortPCHandler()
{
    static double start_time = 1e-3;

    double dist;
    string entity;

    while(start_time < total_time){

        auto frame = lidar->getFrame(start_time);
        lidartype::v_time_pc hits;
        hits.reserve(frame.size());

        auto tic = std::chrono::high_resolution_clock::now();

        for(const auto& f : frame){

            auto dist_lidarInG = sixsp->getPose(start_time);
            auto lidarInG = sixsp->getPose(f.timestamp);
            
            lidartype::ig_pc tmp_pc;
            tmp_pc.timestamp = f.timestamp;
            tmp_pc.ring.reserve(f.ring.size());
            tmp_pc.pc.reserve(f.pc.size());
            
            for(int i=0; i<f.pc.size(); i++){
                const auto& p = f.pc[i];  

                IV3d new_ray = lidarInG.Rot() * p + lidarInG.Pos();
                laserShape->SetPoints(lidarInG.Pos(), new_ray);
                laserShape->GetIntersection(dist, entity);

                if(entity.size()){
                    tmp_pc.ring.push_back(f.ring[i]);

                    lidartype::point_t pt;
                    auto pdir = dist * laserShape->getDir();
                    pt.X() = pdir.X();
                    pt.Y() = pdir.Y();
                    pt.Z() = pdir.Z();
                    tmp_pc.pc.push_back(pt);

                    // save undist pc
                    if(save_pcd){
                        auto d_pt =  dist_lidarInG.Rot() * lidarInG.Rot().Inverse() * pt + dist_lidarInG.Pos();
                        auto ud_pt = pt + lidarInG.Pos();
                        pcl::PointXYZ dp, udp;
                        dp.x = d_pt.X();
                        dp.y = d_pt.Y();
                        dp.z = d_pt.Z();
                        udp.x = ud_pt.X();
                        udp.y = ud_pt.Y();
                        udp.z = ud_pt.Z();
                        dist_pc->push_back(dp);
                        undist_pc->push_back(udp);
                    }
                }
            }
            if(tmp_pc.pc.size())    hits.push_back(tmp_pc);
        }

        lidar->writeToBag(rbag, hits, lidar_topic);

        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration_ms = toc - tic;
        gzmsg << "process: " << start_time / total_time << endl;

        start_time += 1.0 / lidar->getHz();        
    }

    // downsample pcd and save
    if(save_pcd){
        save_pcd = false;
        gzmsg << "saving pcd..." << endl;
        gzmsg << "dist_pc size: " << dist_pc->size() << endl;
        gzmsg << "undist_pc size: " << undist_pc->size() << endl;

        pcl::VoxelGrid<pcl::PointXYZ> vg1, vg2;
        vg1.setLeafSize(0.3, 0.3, 0.3);
        vg1.setInputCloud(dist_pc);
        vg1.filter(*dist_pc);
        pcl::io::savePCDFileBinary(pcd_dir + "dist_pc.pcd", *dist_pc);

        vg2.setLeafSize(0.3, 0.3, 0.3);
        vg2.setInputCloud(undist_pc);
        vg2.filter(*undist_pc);
        pcl::io::savePCDFileBinary(pcd_dir + "undist_pc.pcd", *undist_pc);
    }

}

// gzmsg << "one frame cost: " << duration_ms.count() << "ms" << endl;
// for vis
// sensor_msgs::PointCloud2 ros_pc;
// pcl::toROSMsg(pc, ros_pc);
// ros_pc.header.stamp = ros::Time().fromSec(start_time);
// ros_pc.header.frame_id = "velodyne";
// pc_pub.publish(ros_pc);
// this_thread::sleep_for(std::chrono::milliseconds(100));

} // namespace gazebo
