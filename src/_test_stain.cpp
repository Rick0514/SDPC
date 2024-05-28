#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/RenderingIface.hh>

#include <basic.hpp>
#include <icecream.hpp>
#include <yaml-cpp/yaml.h>
#include <tictoc.hpp>
#include <pcp.hpp>

#include <ros/package.h>

#include <opencv2/opencv.hpp>

using std::pair;
using std::string;
using std::cout;
using std::endl;

string src_dir = "";
string world_name;
YAML::Node yml;

using namespace gazebo;
using IV3d = ignition::math::Vector3d;
using IV3i = ignition::math::Vector3i;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

class OneCam
{
public:
    OneCam(const rendering::ScenePtr& sc_) : scene(sc_) {
        sdf::ElementPtr cameraSDF(new sdf::Element);
        sdf::initFile("camera.sdf", cameraSDF);
        cam = scene->CreateCamera("cam0", false);
        cam->Load(cameraSDF);
        cam->Init();
        cam->SetCaptureData(true);
        cam->SetImageWidth(text_size);
        cam->SetImageHeight(text_size);
        cam->SetHFOV(static_cast<ignition::math::Angle>(IGN_DTOR(90)));
        cam->SetAspectRatio(1.0);
        // cam->SetClipDist(0.1, 200);
        cam->CreateRenderTexture("CamTex");
    }

    void SetPos(const IP6d& pos_)
    {
        cam->SetWorldPose(pos_);
        cam->Update();
    }

    void GetImage()
    {
        cam->Render(true);
        cam->PostRender();
        img = cv::Mat(text_size, text_size, CV_8UC3,
            const_cast<unsigned char*>(cam->ImageData()));
    }

    void ShowImage()
    {
        cv::imshow("show", img);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    IV3d Project(const IV3d& pt)
    {
        static int deb = 0;

        IV3d scr;
        auto posInCam = cam->OgreCamera()->getViewMatrix() * //rendering::Conversions::Convert(pt);
                Ogre::Vector4(rendering::Conversions::Convert(pt));
        // if not convert the pt to 4-dims, ogre3d will do it automatically and 
        // norm the w to 1.0, then discard the w to 3-dim vector
        auto pos = cam->OgreCamera()->getProjectionMatrix() * posInCam;
                
        if(deb == 0){
            cout << "pt: " << pt << endl;
            cout << "posInCam: " << posInCam << endl;
            cout << "pos: " << pos << endl;
            cout << "projection matrix: " << cam->OgreCamera()->getProjectionMatrix() << endl;
            cout << "view matrix: " << cam->OgreCamera()->getViewMatrix() << endl;
        }

        pos.x /= pos.w;
        pos.y /= pos.w;
        
        scr.X() = ((pos.x / 2.0) + 0.5) * cam->ViewportWidth();
        scr.Y() = (-(pos.y / 2.0) + 0.5) * cam->ViewportHeight();
        scr.Z() = pos.z;

        if(deb == 0){
            cout << "pos: " << pos << endl;
            cout << "scr: " << scr << endl;
        }

        deb++;
        return scr;
    }

    bool GetRGB(const IV3d& pt, IV3i& uv, IV3i& rgb)
    {        
        auto scr = Project(pt);
        if(scr.X() >= 0 && int(scr.X()) < text_size && 
            scr.Y() >= 0 && int(scr.Y()) < text_size &&
            scr.Z() > 0.0)
        {
            int u = scr.X();
            int v = scr.Y();
            
            cv::Vec3b data = img.at<cv::Vec3b>(v, u);
            rgb.X() = data[2];
            rgb.Y() = data[1];
            rgb.Z() = data[0];

            uv.X() = u;
            uv.Y() = v;

            return true;
        }

        return false;
    }

    void ShowStainProcess(const vec_t<IV3d>& pc)
    {
        // stain pc
        cv::Mat showimg = img.clone();
        pcrgb_t::Ptr pc_rgb(new pcrgb_t);

        for(const auto& pt : pc){
            IV3i uv, rgb;
            if(GetRGB(pt, uv, rgb)){
                
                prgb_t prgb;
                prgb.x = pt.X();
                prgb.y = pt.Y();
                prgb.z = pt.Z();
                prgb.r = rgb.X();
                prgb.g = rgb.Y();
                prgb.b = rgb.Z();
                pc_rgb->push_back(prgb);
            
                cv::circle(showimg, cv::Point(uv.X(), uv.Y()), 1, cv::Scalar(255, 0, 0), -1);
            }
        }

        pcl::io::savePCDFileBinary(src_dir + "/pcd/pc_rgb.pcd", *pc_rgb);

        cv::imshow("show", showimg);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }


protected:

    rendering::ScenePtr scene;
    rendering::CameraPtr cam;
    const int text_size{720};

    cv::Mat img;
};

class TinyLidar
{
public:
    TinyLidar()
    {
        physics::WorldPtr w = physics::get_world(world_name);
        rays = boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
                w->Physics()->CreateShape("multiray",
                gazebo::physics::CollisionPtr()));
    }

    // now only call once
    vec_t<IV3d> setDirAndShoot(const vec_t<pair<IV3d, IV3d>>& dirs)
    {
        vec_t<IV3d> ret, dd;

        for(const auto& [st, ed] : dirs)
        {
            double norm = (ed - st).Length();
            IV3d d_ = (ed - st) / norm;
            rays->AddRay(st, st + d_ * 200);
            dd.push_back(d_);
        }

        double dist;
        string entity;
        for(int i=0; i<rays->RayCount(); i++){
            entity.clear(); dist = 1e6;
            rays->Ray(i)->GetIntersection(dist, entity);
            if(entity.size() && dist <= 20){
                auto st = rays->Ray(i)->Start();
                ret.push_back(st + dd[i] * dist);
            }
        }
        
        return ret;
    }

    static void savePCD(const vec_t<IV3d>& pc, const string& path)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(pc.size());
        for(int i=0; i<pc.size(); i++){
            cloud->points[i].x = pc[i].X();
            cloud->points[i].y = pc[i].Y();
            cloud->points[i].z = pc[i].Z();
        }
        pcl::io::savePCDFileBinary(path, *cloud);
    }

protected:
    
    gazebo::physics::MultiRayShapePtr rays;
};


int main(int argc, char **argv)
{
    Common_tools::Timer timer;
    gazebo::common::Console::SetQuiet(false);

    // use rospack to find the package path
    src_dir = ros::package::getPath("sdpc"); IC(src_dir);
    std::ifstream inf(src_dir + "/config/mld.yaml");
    yml = YAML::Load(inf);
    
    gazebo::setupServer(argc, argv);  
    
    // Load the world
    string world_fn = src_dir + "/worlds/hatchback"; IC(world_fn);
    gazebo::physics::WorldPtr world = gazebo::loadWorld(world_fn);
    world_name = world->Name();

    // init world's visual
    gazebo::rendering::ScenePtr scene = gazebo::rendering::create_scene(world_name, false, true);
    for(int i=0; i<10; i++){
        event::Events::preRender();
        gazebo::runWorld(world, 100);
    }

    // the whole world is ready
    TinyLidar tyld;
    // prepare the directions, make some parralel lines
    vec_t<pair<IV3d, IV3d>> dirs;
    float ed = 10;
    float step = 1e-1;

    for(float i=-2.0; i<2.0; i+=step){
        for(float j=0.1; j<3.0; j+=step){
            dirs.push_back({IV3d(0, i, j), IV3d(ed, i, j)});
        }
    }
    auto igpc = tyld.setDirAndShoot(dirs);
    // TinyLidar::savePCD(igpc, src_dir + "/pcd/igpc.pcd");

    // prepare cam
    OneCam oc(scene);
    IP6d pose;
    pose.Pos().Set(0, 0, 1.0);
    pose.Rot().Euler().Set(0, IGN_DTOR(0), 0);
    oc.SetPos(pose);
    oc.GetImage();
    oc.ShowStainProcess(igpc);

    IC("finished!");
    gazebo::shutdown();

    return 0;
}
