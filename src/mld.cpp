#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <basic.hpp>
#include <icecream.hpp>
#include <yaml-cpp/yaml.h>
#include <tictoc.hpp>

#include <rosbag/bag.h>

#include <sixdofcubicspline.hpp>
#include <lidartype.hpp>

using std::string;
using std::cout;
using std::endl;

string src_dir = "";

using namespace gazebo;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

class GzLidar
{
public:
    GzLidar(physics::WorldPtr& w, string type, YAML::Node& yml){
        rays = boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
                w->Physics()->CreateShape("multiray",
                gazebo::physics::CollisionPtr()));
        auto yyml = yml["lidar_info"];
        yyml["livox"]["scan_dir"] = src_dir + yyml["livox"]["scan_dir"].as<string>();

        IC();

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

                // if(idx % 100 == 0)   IC(dist);

                // noise currupted
                dist += gn.getNoise(ld->getNoiseStd());
                lidartype::point_t pt = dist * p;
                tmp_pc.ring.push_back(f.ring[i]);
                tmp_pc.pc.push_back(pt);
            }
            if(tmp_pc.pc.size())    hits.push_back(tmp_pc);
        }
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

    MultiLidars();    
    
    void Run();
    
protected:

    string world_fn;
    
    float hz;

    vec_t<string> lidar_types, lidar_topics;
    string odom_topic;
    int lidar_num;

    rosbag::Bag rbag;

    string path_fn;
    double total_time;
    SixDofCubicSpline* sixsp;

    vec_t<GzLidar> glds;

    vec_t<IP6d> igexts;

};

MultiLidars::MultiLidars()
{
    std::ifstream inf(src_dir + "/config/mld.yaml");
    auto yml = YAML::Load(inf);

    world_fn = yml["world_fn"].as<string>();
    world_fn = src_dir + world_fn; IC(world_fn);

    // Load the world
    gazebo::physics::WorldPtr world = gazebo::loadWorld(world_fn);

    lidar_types = yml["lidar_types"].as<vec_t<string>>();
    lidar_topics = yml["lidar_topics"].as<vec_t<string>>();
    odom_topic = yml["odom_topic"].as<string>();
    hz = yml["lidar_info"]["hz"].as<float>();

    lidar_num = lidar_topics.size();

    string bag_fn = src_dir + yml["out_bag"].as<string>();  IC(bag_fn);
    rbag.open(bag_fn, rosbag::bagmode::Write);

    // for path
    path_fn = src_dir + yml["path"]["path_fn"].as<string>();
    total_time = yml["path"]["total_time"].as<double>();
    sixsp = new SixDofCubicSpline(total_time, path_fn);

    for(auto& t : lidar_types){
        glds.push_back(GzLidar(world, t, yml));
    }

    IC();

    // for exts
    auto exts = yml["exts"].as<vector<float>>(); IC(exts);
    assert(exts.size() == 7 * lidar_num);

    for(int i=0; i<lidar_num; i++){
        int idx = 7 * i;
        IV3d pos(exts[idx+4], exts[idx+5], exts[idx+6]);
        IQd rot(exts[idx+3], exts[idx], exts[idx+1], exts[idx+2]);
        igexts.push_back(IP6d(pos, rot));
    }

    IC();
}

void MultiLidars::Run()
{
    double start_time = 0.1;

    while(start_time < total_time){
        
        auto T_w_b = sixsp->getPose(start_time);
        
        for (int i = 0; i < lidar_num; i++)
        {
            auto& ld = glds[i];
            const auto& T_b_li = igexts[i];
            auto T_w_li = T_w_b * T_b_li;

            auto hits = ld.getPointCloud(T_w_li, start_time); IC(hits.size());

            ld.getLidar()->writeToBag(rbag, hits, lidar_topics[i]);
        }
        start_time += 1.0 / hz;
    }

    rbag.close();
}

int main(int argc, char **argv)
{

#ifdef ROOT_DIR
    src_dir = ROOT_DIR;
#endif

    Common_tools::Timer timer;
    
    timer.tic("init");
    gazebo::setupServer(argc, argv);
    MultiLidars mld;
    cout << timer.toc_string("init") << endl;

    timer.tic("run");    
    mld.Run();
    cout << timer.toc_string("run") << endl;

    IC("finished!");

    return 0;
}
