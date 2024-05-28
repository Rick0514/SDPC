#pragma once

#include <string>
#include "basic.hpp"

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"

#include <yaml-cpp/yaml.h>

namespace camera
{

using namespace gazebo;
using IV3i = ignition::math::Vector3i;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

class OneCam
{
public:
    OneCam(const rendering::ScenePtr& sc_, YAML::Node& yml) : scene(sc_) {
        static int cam_num = 0;
        cam_name = std::string("cam") + std::to_string(cam_num);

        float hfov = IGN_DTOR(yml["hfov"].as<float>());
        auto hw = yml["hw"].as<vec_t<int>>();
        auto noise = yml["noise"].as<vec_t<float>>();

        std::ostringstream newModelStr;
        newModelStr << "<sdf version='" << SDF_VERSION << "'>"
            << "      <horizontal_fov>" << hfov << "</horizontal_fov>"
            << "      <image>"
            << "        <width>" << hw[1] << "</width>"
            << "        <height>" << hw[0] << "</height>"
            << "        <format>R8G8B8</format>"
            << "      </image>"
            << "      <noise>"
            << "        <type>gaussian</type>"
            << "        <mean>" << noise[0] << "</mean>"
            << "        <stddev>" << noise[1] << "</stddev>"
            << "      </noise>";

        sdf::ElementPtr cameraSDF(new sdf::Element);
        sdf::initFile("camera.sdf", cameraSDF);
        sdf::readString(newModelStr.str(), cameraSDF);     

        cam = scene->CreateCamera("cam0", false);
        cam->Load(cameraSDF);
        cam->Init();
        cam->SetCaptureData(true);
        cam->CreateRenderTexture("CamTex" + std::to_string(cam_num++));
    
        vec_t<float> tbc_vec = yml["ext"].as<vec_t<float>>();
        IV3d pos(tbc_vec[4], tbc_vec[5], tbc_vec[6]);
        IQd rot(tbc_vec[3], tbc_vec[0], tbc_vec[1], tbc_vec[2]);
        Tbc = IP6d(pos, rot);
    }

    IP6d GetTbc() const {   return Tbc;     }

    std::string GetCamName() const { return cam_name; }

    void SetPos(const IP6d& pos_)
    {
        IP6d Twc = Tbc + pos_;
        cam->SetWorldPose(Twc);
        cam->Update();
    }

    void GetImage()
    {
        cam->Render(true);
        cam->PostRender();
    }

    void WriteToBag(rosbag::Bag& bag, double t, std::string topic)
    {
        GetImage();
        // convert the image to sensor_msgs::Image
        sensor_msgs::Image msg;
        msg.header.frame_id = cam_name;
        msg.header.stamp = ros::Time().fromSec(t);
        msg.encoding = "rgb8";
        msg.height = cam->ImageHeight();
        msg.width = cam->ImageWidth();
        msg.step = cam->ImageWidth() * 3;
        msg.is_bigendian = false;

        size_t image_size = msg.height * msg.step;
        msg.data.resize(image_size);
        memcpy(&msg.data[0], cam->ImageData(), image_size);

        bag.write(topic, msg.header.stamp, msg);
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
                
        pos.x /= pos.w;
        pos.y /= pos.w;
        
        scr.X() = ((pos.x / 2.0) + 0.5) * cam->ViewportWidth();
        scr.Y() = (-(pos.y / 2.0) + 0.5) * cam->ViewportHeight();
        scr.Z() = pos.z;

        return scr;
    }

protected:

    rendering::ScenePtr scene;
    rendering::CameraPtr cam;

    std::string cam_name;
    IP6d Tbc;
};

}

