#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/RenderingIface.hh>

#include <basic.hpp>
#include <icecream.hpp>
#include <yaml-cpp/yaml.h>
#include <tictoc.hpp>
#include <pcp.hpp>

#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>

#include <sixdofcubicspline.hpp>
#include <lidartype.hpp>
#include <cubemap.hpp>
#include <camera.hpp>
#include <imu.hpp>

INIT_IMU_CLASS;
INIT_CAMERA_CLASS;
INIT_CUBEMAP_CLASS;

using std::string;
using std::cout;
using std::endl;

string g_world_name;
string g_src_dir = "";
string g_task = "record";
YAML::Node g_yml;

using namespace gazebo;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

class AddNoiseToPose
{
public:

    AddNoiseToPose() {}
    AddNoiseToPose(double ang_, double trans_) : ang_noise(ang_), trans_noise(trans_) {} 

    void setNoise(double ang_, double trans_){
        // set ang_noise and trans_noise
        ang_noise = ang_;
        trans_noise = trans_;
    }

    double getNoise(double mu, double std)
    {
        std::normal_distribution<double> d(mu, std);
        return d(gen);
    }

    double getNoise(double std)
    {
        std::normal_distribution<double> d(0.0, std);
        return d(gen);
    }

    IP6d noisePose(const IP6d& p)
    {
        // add noise to p
        auto pos = p.Pos();
        auto rot = p.Rot();

        // add noise to pos
        IV3d axis;
        double ang;
        rot.ToAxis(axis, ang);
        ang += getNoise(ang_noise);
        rot = IQd(axis, ang);

        for(int i=0; i<3; i++){
            pos[i] += getNoise(trans_noise);
        }

        return IP6d(pos, rot);
    }

    static nav_msgs::Odometry IgToOdomMsg(const IP6d& po, double tt, string frame_id){
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(tt);
        odom.header.frame_id = "map";
        odom.child_frame_id = frame_id;
        auto p = po.Pos();
        auto q = po.Rot();
        odom.pose.pose.position.x = p.X();
        odom.pose.pose.position.y = p.Y();
        odom.pose.pose.position.z = p.Z();
        odom.pose.pose.orientation.w = q.W();
        odom.pose.pose.orientation.x = q.X();
        odom.pose.pose.orientation.y = q.Y();
        odom.pose.pose.orientation.z = q.Z();
        return odom;
    }


protected:
    std::random_device rd{};
    std::mt19937 gen{rd()};

    double ang_noise, trans_noise;
};

class GzLidar
{
public:
    GzLidar(physics::WorldPtr& w, string type){
        rays = boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
                w->Physics()->CreateShape("multiray",
                gazebo::physics::CollisionPtr()));

        auto yyml = g_yml["lidar_info"];
        IC(yyml["livox"]["scan_dir"].as<string>());

        ld = lidartype::LidarBase::create(type, yyml);

        IC();

        // init rays
        auto frame = ld->getFrame(0);
        for(const auto& f : frame){
            for(int i=0; i<f.pc.size(); i++){
                const auto& pt = f.pc[i];
                rays->AddRay(pt * ld->getRange().first, pt * ld->getRange().second);
            }
        }
    }
    
    lidartype::v_time_pc getPointCloud(const IP6d& po, double tt)
    {
        auto frame = ld->getFrame(tt);

        lidartype::v_time_pc hits;
        hits.reserve(frame.size());
        double dist;
        string entity;
        GenNoise gn;
        
        int idx = 0;
        double range = ld->getRange().second - ld->getRange().first;

        int pc_num = 0;
        for(const auto& f : frame){
            lidartype::ig_pc tmp_pc;
            tmp_pc.timestamp = f.timestamp;
            tmp_pc.ring.reserve(f.ring.size());
            tmp_pc.pc.reserve(f.pc.size());

            for(int i=0; i<f.pc.size(); i++){
                const auto& p = f.pc[i];
                IV3d st_ray = po.Rot() * p * ld->getRange().first + po.Pos();
                IV3d ed_ray = po.Rot() * p * ld->getRange().second + po.Pos();
                rays->Ray(idx)->SetLength(range);
                rays->Ray(idx)->SetPoints(st_ray, ed_ray);

                // reset
                entity.clear(); dist = 1e6;
                rays->Ray(idx)->GetIntersection(dist, entity);
                
                idx++;
                if(entity.empty() || dist > range)    continue;

                // noise currupted
                double ns = gn.getNoise(ld->getNoiseStd());
                dist += ns;
                lidartype::point_t pt = dist * p;
                                
                tmp_pc.ring.push_back(f.ring[i]);
                tmp_pc.pc.push_back(pt);
                pc_num++;

            }
            if(tmp_pc.pc.size())    hits.push_back(tmp_pc);
        }
        IC(ld->name, pc_num);
        return hits;
    }

    std::shared_ptr<lidartype::LidarBase> getLidar() { return ld; }

protected:

    std::shared_ptr<lidartype::LidarBase> ld;
    gazebo::physics::MultiRayShapePtr rays;
};

class MultiLidars
{
public:

    MultiLidars(rendering::ScenePtr scene_);    
    
    void RecordDataset();

    void ShotAtPoses();

    void MakePriorMap();

    static void parseTUM(const string& fn, vec_t<IP6d>& poses)
    {
        std::ifstream inf(fn);
        if(!inf.is_open()){
            cout << "file not found: " << fn;
            return;
        }

        string line;
        while(std::getline(inf, line)){
            std::istringstream iss(line);
            double t;
            IV3d pos;
            IQd rot;
            iss >> t >> pos.X() >> pos.Y() >> pos.Z() >> rot.X() >> rot.Y() >> rot.Z() >> rot.W();
            poses.push_back(IP6d(pos, rot));
        }
    }
 
protected:

    string world_fn;
    
    float hz;

    vec_t<string> lidar_types, lidar_topics;
    string odom_topic, gt_odom_topic;
    int lidar_num;

    rosbag::Bag rbag;

    string path_fn;
    double total_time;
    SixDofCubicSpline* sixsp;

    vec_t<GzLidar> glds;

    vec_t<IP6d> igexts;

    vec_t<double> odom_noise;

    // for shot
    string shot_dir, shot_fn;
    vec_t<IP6d> shot_poses;

    GzLidar* mkpm_lidar;
    string mkpm_dir, mkpm_fn, save_fn;
    int num_each_loc;
    float mkpm_ds;

    bool stain_en;
    vec_t<IP6d> stain_pos;

    string cam_fn, imu_fn;

    rendering::ScenePtr scene;
};

MultiLidars::MultiLidars(rendering::ScenePtr scene_) : scene(scene_)
{
    physics::WorldPtr world = physics::get_world(g_world_name);

    lidar_types = g_yml["lidar_types"].as<vec_t<string>>(); IC(lidar_types);
    lidar_topics = g_yml["lidar_topics"].as<vec_t<string>>(); IC(lidar_topics);
    odom_topic = g_yml["odom_topic"].as<string>(); IC(odom_topic);
    gt_odom_topic = g_yml["gt_odom_topic"].as<string>(); IC(gt_odom_topic);
    odom_noise = g_yml["odom_noise"].as<vec_t<double>>(); IC(odom_noise);
    hz = g_yml["lidar_info"]["hz"].as<float>();

    lidar_num = lidar_topics.size();

    string bag_fn = g_src_dir + g_yml["out_bag"].as<string>();  IC(bag_fn);
    rbag.open(bag_fn, rosbag::bagmode::Write);

    g_yml["lidar_info"]["livox"]["scan_dir"] = g_src_dir + g_yml["lidar_info"]["livox"]["scan_dir"].as<string>();

    // for path
    path_fn = g_src_dir + g_yml["path"]["path_fn"].as<string>();
    total_time = g_yml["path"]["total_time"].as<double>();
    sixsp = new SixDofCubicSpline(total_time, path_fn);

    if(g_yml["cam_yml"].IsDefined()){
        cam_fn = g_src_dir + g_yml["cam_yml"].as<string>(); IC(cam_fn);
    }

    if(g_yml["imu_yml"].IsDefined()){
        imu_fn = g_src_dir + g_yml["imu_yml"].as<string>(); IC(imu_fn);
    }

    for(auto& t : lidar_types){
        glds.push_back(GzLidar(world, t));
    }

    if(g_task == "shot"){
        shot_dir = g_yml["shot"]["shot_dir"].as<string>();
        shot_fn = shot_dir + g_yml["shot"]["shot_fn"].as<string>();
        parseTUM(shot_fn, shot_poses); IC(shot_poses.size());
    }

    if(g_task == "mkpm"){
        IC();
        mkpm_dir = g_yml["mkpm"]["mkpm_dir"].as<string>();
        mkpm_fn = mkpm_dir + g_yml["mkpm"]["mkpm_fn"].as<string>();
        save_fn = mkpm_dir + g_yml["mkpm"]["save_fn"].as<string>();
        string mkpm_ld_type = g_yml["mkpm"]["lidar_type"].as<string>();
        num_each_loc = g_yml["mkpm"]["num_each_loc"].as<int>();

        IC(mkpm_fn, num_each_loc, mkpm_ld_type);
        mkpm_ds = g_yml["mkpm"]["ds"].as<float>();
        mkpm_lidar = new GzLidar(world, mkpm_ld_type);

        stain_en = g_yml["mkpm"]["stain"]["enable"].as<bool>();
        string cm_fn = mkpm_dir + g_yml["mkpm"]["stain"]["cm_fn"].as<string>();
        parseTUM(cm_fn, stain_pos);
    }

    IC();

    // for exts
    auto exts = g_yml["exts"].as<vector<float>>(); IC(exts);
    assert(exts.size() == 7 * lidar_num);

    for(int i=0; i<lidar_num; i++){
        int idx = 7 * i;
        IV3d pos(exts[idx+4], exts[idx+5], exts[idx+6]);
        IQd rot(exts[idx+3], exts[idx], exts[idx+1], exts[idx+2]);
        igexts.push_back(IP6d(pos, rot));
    }

    IC();
}

void MultiLidars::RecordDataset()
{
    double start_time = 0.1;
    double tt = start_time;
    double dur = 1.0 / hz;

    static AddNoiseToPose antp(odom_noise[0] * Const::to_rad, odom_noise[1]);

    while(tt < total_time)
    {
        auto T_w_b = sixsp->getPose(tt);
        // convert T_w_b to Odometry and save to rbag
        auto T_w_b_noise = antp.noisePose(T_w_b);
        nav_msgs::Odometry odom = AddNoiseToPose::IgToOdomMsg(T_w_b_noise, tt, "base_link");
        rbag.write(odom_topic, ros::Time(tt), odom);

        // gt odom
        nav_msgs::Odometry gt_odom = AddNoiseToPose::IgToOdomMsg(T_w_b, tt, "gt_base_link");
        rbag.write(gt_odom_topic, ros::Time(tt), gt_odom);

        for (int i = 0; i < lidar_num; i++)
        {
            auto& ld = glds[i];
            const auto& T_b_li = igexts[i];
            // anti-human api, the order reverse from our normal sense
            auto T_w_li = T_b_li + T_w_b;

            auto hits = ld.getPointCloud(T_w_li, tt);
            ld.getLidar()->writeToBag(rbag, hits, lidar_topics[i]);
        }
        tt += dur;
    }


    if(cam_fn.size()){
        // need to record image data
        tt = start_time;
        auto cam_yml = YAML::LoadFile(cam_fn);
        auto cam_hz = cam_yml["hz"].as<int>();
        dur = 1.0 / cam_hz;

        // init some cams
        vec_t<camera::OneCam> cams;
        for(int i=0; i<5; i++)  // we assume 5 cams is the max
        {
            string cam_name = "cam" + std::to_string(i);
            if(!cam_yml[cam_name].IsDefined())  break;

            auto cyml = cam_yml[cam_name];
            cams.push_back(camera::OneCam(scene, cyml));
        }

        while(tt < total_time){
            auto T_w_b = sixsp->getPose(tt);

            for(auto& cam : cams){
                cam.SetPos(T_w_b);
                cam.WriteToBag(rbag, tt, cam.GetCamName());
            }

            tt += dur;
        }
    }

    if(imu_fn.size())
    {
        // need to record imu data
        tt = start_time;
        auto imu_yml = YAML::LoadFile(imu_fn);
        auto imu_hz = imu_yml["hz"].as<int>();
        dur = 1.0 / imu_hz;
        IC(dur, imu_hz);

        // init imu static member
        imu::Imu::imu_hz = imu_hz;
        auto grav_vec = imu_yml["grav"].as<vec_t<float>>();
        imu::Imu::grav = IV3d(grav_vec[0], grav_vec[1], grav_vec[2]);

        vec_t<imu::Imu> imus;
        for(int i=0; i<5; i++)
        {
            string imu_name = "imu" + std::to_string(i);
            if(!imu_yml[imu_name].IsDefined())  break;

            auto iyml = imu_yml[imu_name];
            imus.push_back(imu::Imu(iyml));
        }

        IC(imus.size());
        
        while(tt < total_time)
        {
            // get the pose
            auto Twb = sixsp->getPose(tt);
            auto v = sixsp->getV(tt);
            auto dv = sixsp->getA(tt);
            auto w = sixsp->getOmega(tt);
            auto dw = sixsp->getAlpha(tt);

            for(auto& imu : imus)
            {
                imu.FromBaseKin(Twb, v, dv, w, dw);
                imu.WriteToBag(rbag, tt, imu.GetImuName());
            }

            tt += dur;
        }
    }

    rbag.close();
}

void MultiLidars::ShotAtPoses()
{
    double tt = 0.1;
    double dur = 1.0 / hz;

    for(auto& T_w_b : shot_poses)
    {
        nav_msgs::Odometry odom = AddNoiseToPose::IgToOdomMsg(T_w_b, tt, "base_link");
        rbag.write(odom_topic, ros::Time(tt), odom);

        odom = AddNoiseToPose::IgToOdomMsg(T_w_b, tt, "base_link");
        rbag.write(gt_odom_topic, ros::Time(tt), odom);

        for (int i = 0; i < lidar_num; i++)
        {
            auto& ld = glds[i];
            const auto& T_b_li = igexts[i];
            // anti-human api, the order reverse from our normal sense
            auto T_w_li = T_b_li + T_w_b;

            auto hits = ld.getPointCloud(T_w_li, tt);
            ld.getLidar()->writeToBag(rbag, hits, lidar_topics[i]);
        }

        tt += dur;
    }

    rbag.close();
}

void MultiLidars::MakePriorMap()
{
    vec_t<IP6d> poses;
    parseTUM(mkpm_fn, poses);

    pc_ptr gmap = pcl::make_shared<pc_t>();
    VoxelDownSample vds(mkpm_ds);
    for(auto& T_w_b : poses)
    {
        pc_t pc;
        for (int i = 0; i < num_each_loc; i++)
        {
            auto hits = mkpm_lidar->getPointCloud(T_w_b, 0);
            for(const auto& p : hits){
                auto pp = p.pc[0]; 
                pt_t pt;
                pt.intensity = 100.0;
                pp = T_w_b.Rot() * pp + T_w_b.Pos();
                pt.x = pp.X();
                pt.y = pp.Y();
                pt.z = pp.Z();
                pc.emplace_back(std::move(pt));
            }
        }
        // vds.filter<pt_t>(gmap, gmap);
        *gmap += pc;
    }
    vds.filter<pt_t>(gmap, gmap);

    if(!stain_en){
        savePCD(save_fn, *gmap); IC(save_fn, gmap->size());
    }else{
        int text_size = g_yml["mkpm"]["stain"]["text_size"].as<int>();

        vector<cubemap::CubeMap> cms;
        for(const auto& po : stain_pos){
            cubemap::CubeMap cm(scene, text_size);
            cm.SetPos(po.Pos());
            cm.GetCubeMap();
            cms.push_back(cm);
        }

        pcrgb_t::Ptr pc_rgb(new pcrgb_t);
        pc_rgb->resize(gmap->size());
        for(int i=0; i<gmap->size(); i++)
        {
            auto& pt = gmap->points[i];
            auto& prgb = pc_rgb->points[i];
            
            vector<double> dist;
            IV3d igpt(pt.x, pt.y, pt.z);
            for(const auto& c : cms){
                dist.push_back((igpt - c.GetPos()).Length());
            }

            // get the min idx of dist
            auto min_it = std::min_element(dist.begin(), dist.end());
            int min_idx = std::distance(dist.begin(), min_it);

            auto& chosen_cm = cms[min_idx];
            auto rgb = chosen_cm.GetRGB(IV3d(pt.x, pt.y, pt.z));
            prgb.x = pt.x;
            prgb.y = pt.y;
            prgb.z = pt.z;
            prgb.r = rgb.X();
            prgb.g = rgb.Y();
            prgb.b = rgb.Z();
        }

        savePCD(save_fn, *pc_rgb); IC(save_fn, pc_rgb->size());
    }
}

int main(int argc, char **argv)
{

#ifdef ROOT_DIR
    g_src_dir = ROOT_DIR;
#endif

    Common_tools::Timer timer;
    gazebo::common::Console::SetQuiet(false);
    
    std::ifstream inf(g_src_dir + "/config/mld.yaml");
    g_yml = YAML::Load(inf);

    timer.tic("init");
    gazebo::setupServer(argc, argv);
    string world_fn = g_yml["world_fn"].as<string>();
    g_task = g_yml["task"].as<string>();
    world_fn = g_src_dir + world_fn; IC(g_task, world_fn);
    // Load the world
    gazebo::physics::WorldPtr world = gazebo::loadWorld(world_fn);
    g_world_name = world->Name();
    gazebo::rendering::ScenePtr scene = gazebo::rendering::create_scene(g_world_name, false, true);
    for(int i=0; i<5; i++){
        event::Events::preRender();
        gazebo::runWorld(world, 100);
    }
    cout << timer.toc_string("init") << endl;

    MultiLidars mld(scene);

    timer.tic("g_task");

    if(g_task == "record")
        mld.RecordDataset();
    else if(g_task == "shot")
        mld.ShotAtPoses();
    else if(g_task == "mkpm")
        mld.MakePriorMap();
    else
        cout << "No such g_task!!" << endl;

    cout << timer.toc_string("g_task") << endl;

    IC("finished!");

    return 0;
}
