
#include <chrono>
#include <onerayshape.hpp>
#include "gazebo/physics/physics.hh"
#include <distortpc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace gazebo
{
using std::endl;

GZ_REGISTER_SENSOR_PLUGIN(DistortPC)

pcl::VoxelGrid<pcl::PointXYZ>::Ptr vgptr;

static double gaussianKernel(double mu, double sigma)
{
    // using Box-Muller transform to generate two independent standard normally distributed normal variables
    // see wikipedia
    double U = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
    double V = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
    return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
}

DistortPC::DistortPC(){
    vgptr = pcl::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
    vgptr->setLeafSize(0.3, 0.3, 0.3);
}

DistortPC::~DistortPC()
{
    parentSensor.reset();
    world.reset();
    laserShape.reset();
}

template <typename T>
void setIVd(IV3d& iv, const vector<T>& v){
    iv.X() = v[0];
    iv.Y() = v[1];
    iv.Z() = v[2];
}

void DistortPC::Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf)
{
    gzmsg << "Load distort lidar plugin now!!" << endl;
    rn = boost::make_shared<ros::NodeHandle>();

    rn->getParam("total_time", total_time);
    rn->getParam("imu_hz", imu_hz);
    rn->getParam("path_fn", path_fn);
    rn->getParam("lidar_topic", lidar_topic);
    rn->getParam("save_bag", save_bag);
    rn->getParam("lidar_type", lidar_type);
    rn->getParam("save_pcd", save_pcd);
    rn->getParam("pcd_dir", pcd_dir);
    
    // imu
    rn->getParam("imu_topic", imu_topic);
    rn->getParam("imu_bg", imu_bg);
    rn->getParam("imu_ba", imu_ba);
    rn->getParam("imu_ng", imu_ng);
    rn->getParam("imu_na", imu_na);

    string scan_dir;
    rn->getParam("scan_dir", scan_dir);

    vector<float> v_tLI, v_rLI, v_g;
    IV3d tmp;
    rn->getParam("grav", v_g);
    rn->getParam("t_LI", v_tLI);
    rn->getParam("r_LI", v_rLI);
    // init ext and grav
    setIVd(tLI, v_tLI);
    setIVd(tmp, v_rLI);
    setIVd(grav, v_g);
    rLI.Euler(tmp);
    
    gzmsg << "path_fn: " << path_fn << endl;
    gzmsg << "save_pcd: " << save_pcd << endl;
    gzmsg << "pcd_dir: " << pcd_dir << endl;

    pc_pub = rn->advertise<sensor_msgs::PointCloud2>(lidar_topic, 5);
    sixsp = new SixDofCubicSpline(total_time, path_fn);

    if(lidar_type == "velodyne"){
        lidar = new lidartype::Velodyne(sdf);
    }else if(lidar_type == "avia"){
        lidar = new lidartype::Avia(sdf, scan_dir);
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

    // save imu first
    saveIMU();

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

// save real pose, v, a, w
void DistortPC::saveIMU()
{
    double start_time = min_start_time;
    double dt = 1.0 / imu_hz;
    double cof = 1.0 / std::sqrt(dt);

    nav_msgs::Odometry gt_pose;
    sensor_msgs::Imu imu;
    imu.header.frame_id = "imu";
    gt_pose.header.frame_id = "map";
    gt_pose.child_frame_id = "gt";

    while(start_time < total_time)
    {
        auto imuInG = sixsp->getPose(start_time);
        auto vel = sixsp->getV(start_time);
        auto acc = sixsp->getA(start_time) + grav;
        auto g_inImu = imuInG.Rot().Inverse();
        acc = g_inImu * acc;
        auto omg = g_inImu * sixsp->getOmega(start_time);
        
        gt_pose.header.stamp.fromSec(start_time);
        gt_pose.pose.pose.position.x = imuInG.Pos().X();
        gt_pose.pose.pose.position.y = imuInG.Pos().Y();
        gt_pose.pose.pose.position.z = imuInG.Pos().Z();
        gt_pose.pose.pose.orientation.x = imuInG.Rot().X();
        gt_pose.pose.pose.orientation.y = imuInG.Rot().Y();
        gt_pose.pose.pose.orientation.z = imuInG.Rot().Z();
        gt_pose.pose.pose.orientation.w = imuInG.Rot().W();
        gt_pose.twist.twist.linear.x = vel.X();
        gt_pose.twist.twist.linear.y = vel.Y();
        gt_pose.twist.twist.linear.z = vel.Z();
        gt_pose.twist.twist.angular.x = omg.X();
        gt_pose.twist.twist.angular.y = omg.Y();
        gt_pose.twist.twist.angular.z = omg.Z();
        
        // add noise
        imu.angular_velocity.x = omg.X() + imu_bg + imu_ng * cof * gaussianKernel(0.0, 1.0);
        imu.angular_velocity.y = omg.Y() + imu_bg + imu_ng * cof * gaussianKernel(0.0, 1.0);
        imu.angular_velocity.z = omg.Z() + imu_bg + imu_ng * cof * gaussianKernel(0.0, 1.0);
        imu.linear_acceleration.x = acc.X() + imu_ba + imu_na * cof * gaussianKernel(0.0, 1.0);
        imu.linear_acceleration.y = acc.Y() + imu_ba + imu_na * cof * gaussianKernel(0.0, 1.0);
        imu.linear_acceleration.z = acc.Z() + imu_ba + imu_na * cof * gaussianKernel(0.0, 1.0);

        imu.header.stamp.fromSec(start_time);
        rbag.write(imu_topic, imu.header.stamp, imu);
        rbag.write("gt_pose", gt_pose.header.stamp, gt_pose);
        start_time += dt;
    }    

    gzmsg << "imu and gt are saved!!" << endl;
}

void DistortPC::DistortPCHandler()
{
    static double start_time = min_start_time;
    
    double dist;
    string entity;

    ignition::math::Pose3d lidarInImu(tLI, rLI);
    nav_msgs::Odometry lidar_pose;
    lidar_pose.header.frame_id = "map";
    lidar_pose.child_frame_id = "gt_lidar";

    while(start_time < total_time){

        auto frame = lidar->getFrame(start_time);
        lidartype::v_time_pc hits;
        hits.reserve(frame.size());

        auto tic = std::chrono::high_resolution_clock::now();
        
        auto dist_lidarInG = sixsp->getPose(start_time) * lidarInImu;

        for(const auto& f : frame){

            auto lidarInG = sixsp->getPose(f.timestamp) * lidarInImu;
            
            lidartype::ig_pc tmp_pc;
            tmp_pc.timestamp = f.timestamp;
            tmp_pc.ring.reserve(f.ring.size());
            tmp_pc.pc.reserve(f.pc.size());
            
            for(int i=0; i<f.pc.size(); i++){
                const auto& p = f.pc[i];  

                IV3d new_ray = lidarInG.Rot() * p + lidarInG.Pos();
                laserShape->SetPoints(lidarInG.Pos(), new_ray);
                laserShape->GetIntersection(dist, entity);

                // noise currupted
                dist += gaussianKernel(0.0, lidar->getNoiseStd());
                double ra = dist / lidar->getMaxRange();

                if(entity.size() && ra <= 1.0){
                    tmp_pc.ring.push_back(f.ring[i]);

                    lidartype::point_t pt = ra * p;
                    tmp_pc.pc.push_back(pt);

                    // save undist pc
                    if(save_pcd){
                        auto d_pt = dist_lidarInG.Rot() * pt + dist_lidarInG.Pos();
                        auto ud_pt = lidarInG.Rot() * pt + lidarInG.Pos();
                        pcl::PointXYZ dp, udp;
                        dp.x = d_pt.X();
                        dp.y = d_pt.Y();
                        dp.z = d_pt.Z();
                        udp.x = ud_pt.X();
                        udp.y = ud_pt.Y();
                        udp.z = ud_pt.Z();
                        dist_pc->push_back(dp);
                        undist_pc->push_back(udp);

                        if(dist_pc->size() > 5e6){
                            vgptr->setInputCloud(dist_pc);
                            vgptr->filter(*dist_pc);

                            vgptr->setInputCloud(undist_pc);
                            vgptr->filter(*undist_pc);

                            gzmsg << "after dist size: " << dist_pc->size() << endl;
                            gzmsg << "after undist size: " << undist_pc->size() << endl;
                        }
                    }
                }
            }
            if(tmp_pc.pc.size())    hits.push_back(tmp_pc);
        }

        lidar_pose.header.stamp.fromSec(start_time);
        lidar_pose.pose.pose.position.x = dist_lidarInG.Pos().X();
        lidar_pose.pose.pose.position.y = dist_lidarInG.Pos().Y();
        lidar_pose.pose.pose.position.z = dist_lidarInG.Pos().Z();
        lidar_pose.pose.pose.orientation.x = dist_lidarInG.Rot().X();
        lidar_pose.pose.pose.orientation.y = dist_lidarInG.Rot().Y();
        lidar_pose.pose.pose.orientation.z = dist_lidarInG.Rot().Z();
        lidar_pose.pose.pose.orientation.w = dist_lidarInG.Rot().W();
        rbag.write("lidar_pose", lidar_pose.header.stamp, lidar_pose);

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
        
        vgptr->setInputCloud(dist_pc);
        vgptr->filter(*dist_pc);
        pcl::io::savePCDFileBinary(pcd_dir + "dist_pc.pcd", *dist_pc);

        vgptr->setInputCloud(undist_pc);
        vgptr->filter(*undist_pc);
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
