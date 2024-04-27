#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include <basic.hpp>
#include <icecream.hpp>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>

#include <sixdofcubicspline.hpp>
#include <lidartype.hpp>

using std::string;
string src_dir = "";
static constexpr float Nan = 1000;

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

        ld = lidartype::LidarBase::create(type, yyml);

        // init rays
        auto frame = ld->getFrame(0);
        for(const auto& f : frame){
            for(int i=0; i<f.pc.size(); i++){
                const auto& pt = f.pc[i];
                rays->AddRay(IV3d(0, 0, 0), pt);
            }
        }
    }
    
    lidartype::v_time_pc getPointCloud(const IP6d& po, double tt)
    {
        auto frame = ld->getFrame(tt);
        int idx = 0;
        for(const auto& f : frame){
            for(int i=0; i<f.pc.size(); i++){
                const auto& pt = f.pc[i];
                IV3d new_ray = po.Rot() * pt + po.Pos();
                rays->Ray(idx++)->SetPoints(po.Pos(), new_ray);
                // reset
                rays->Ray(idx)->SetLength(Nan);
            }
        }
        rays->UpdateRays();

        lidartype::v_time_pc hits;
        hits.reserve(frame.size());
        idx = 0;
        double dist;
        GenNoise gn;
        for(const auto& f : frame){
            lidartype::ig_pc tmp_pc;
            tmp_pc.timestamp = f.timestamp;
            tmp_pc.ring.reserve(f.ring.size());
            tmp_pc.pc.reserve(f.pc.size());

            for(int i=0; i<f.pc.size(); i++){
                const auto& p = f.pc[i];
                IV3d new_ray = po.Rot() * p + po.Pos();
                dist = rays->Ray(idx++)->GetLength();

                // noise currupted
                dist += gn.getNoise(ld->getNoiseStd());
                double ra = dist / ld->getMaxRange();

                if(ra <= 1.0){
                    lidartype::point_t pt = ra * p;
                    tmp_pc.ring.push_back(f.ring[i]);
                    tmp_pc.pc.push_back(pt);
                }
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

class MultiLidars : public ServerFixture
{
public:

    MultiLidars();    
    
    void Run();
    
    void TestBody() override {}

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

    // Load the shapes world
    Load(world_fn, true, "ode");
    physics::WorldPtr world = physics::get_world("default");

    lidar_types = yml["lidar_types"].as<vec_t<string>>();
    lidar_topics = yml["lidar_topics"].as<vec_t<string>>();
    odom_topic = yml["odom_topic"].as<string>();
    hz = yml["lidar_info"]["hz"].as<float>();

    lidar_num = lidar_topics.size();

    string bag_fn = src_dir + yml["out_bag"].as<string>();  IC(bag_fn);
    rbag.open(bag_fn, rosbag::bagmode::Read);

    // for path
    path_fn = src_dir + yml["path"]["path_fn"].as<string>();
    total_time = yml["path"]["total_time"].as<double>();
    sixsp = new SixDofCubicSpline(total_time, path_fn);

    for(auto& t : lidar_types){
        glds.push_back(GzLidar(world, t, yml));
    }

    // for exts
    auto exts = yml["exts"].as<vector<float>>();
    assert(exts.size() == 7 * lidar_num);

    for(int i=0; i<lidar_num; i++){
        int idx = 7 * i;
        IV3d pos(exts[idx+4], exts[idx+5], exts[idx+6]);
        IQd rot(exts[idx+3], exts[idx], exts[idx+1], exts[idx+2]);
        igexts.push_back(IP6d(pos, rot));
    }
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

            auto hits = ld.getPointCloud(T_w_li, start_time);
            ld.getLidar()->writeToBag(rbag, hits, lidar_topics[i]);
        }
        start_time += 1.0 / hz;
    }
}

int main(int argc, char **argv)
{

#ifdef ROOT_DIR
    src_dir = ROOT_DIR;
#endif

    MultiLidars mld;
    mld.Run();

    return 0;
}
