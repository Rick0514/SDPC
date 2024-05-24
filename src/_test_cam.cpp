#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <cubemap.hpp>

#include <basic.hpp>
#include <icecream.hpp>
#include <yaml-cpp/yaml.h>
#include <tictoc.hpp>
#include <pcp.hpp>

#include <ros/package.h>

using std::string;
using std::cout;
using std::endl;

string src_dir = "";
YAML::Node yml;

using namespace gazebo;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

int main(int argc, char **argv)
{
    Common_tools::Timer timer;
    gazebo::common::Console::SetQuiet(false);

    // use rospack to find the package path
    src_dir = ros::package::getPath("sdpc"); IC(src_dir);
    std::ifstream inf(src_dir + "/config/mld.yaml");
    yml = YAML::Load(inf);
    string world_fn = yml["world_fn"].as<string>();
    world_fn = src_dir + world_fn; IC(world_fn);

    gazebo::setupServer(argc, argv);  
    // Load the world
    gazebo::physics::WorldPtr world = gazebo::loadWorld(world_fn);
   
    // init world's visual
    gazebo::rendering::ScenePtr scene = gazebo::rendering::create_scene(world->Name(), false, true);
    for(int i=0; i<3; i++){
        event::Events::preRender();
        gazebo::runWorld(world, 10);
    }

    // the whole world is ready
    cubemap::CubeMap cm(scene, 512);
    cm.SetPos(ignition::math::Vector3d(0, 0, 1));
    cm.GetCubeMap();

    IC("finished!");

    gazebo::shutdown();

    return 0;
}
