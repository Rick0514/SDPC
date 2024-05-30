#pragma once

#include <string>
#include "basic.hpp"

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"

using std::string;
using namespace gazebo;

namespace cubemap
{

using IV3i = ignition::math::Vector3i;
using IV3d = ignition::math::Vector3d;
using IQd = ignition::math::Quaterniond;
using IP6d = ignition::math::Pose3d;

#define INIT_CUBEMAP_CLASS          \
int cubemap::CubeMap::_class_cnt = 0;   \

class CubeMap
{
public:
    static int _class_cnt;
    
    CubeMap(rendering::ScenePtr scene_, int text_size_) : scene(scene_), text_size(text_size_)
    {
        cubes.resize(6);

        sdf::ElementPtr cameraSDF(new sdf::Element);
        sdf::initFile("camera.sdf", cameraSDF);

        string cam_name_prefix = string("cubecam") + std::to_string(_class_cnt) + "_";
        string rt_name_prefix = string("rt") + std::to_string(_class_cnt) + "_";   
        _class_cnt++;

        for(int i=0; i<6; i++){
            cubes[i] = scene->CreateCamera(cam_name_prefix + std::to_string(i), false);
            cubes[i]->Load(cameraSDF);
            cubes[i]->Init();

            // settings
            cubes[i]->SetCaptureData(true);
            cubes[i]->SetImageWidth(text_size);
            cubes[i]->SetImageHeight(text_size);
            cubes[i]->SetHFOV(static_cast<ignition::math::Angle>(IGN_DTOR(95)));
            cubes[i]->SetAspectRatio(1.0);
            cubes[i]->SetClipDist(0.1, 200);
            cubes[i]->CreateRenderTexture(rt_name_prefix + std::to_string(i));
        }
    }

    void SetPos(const IV3d& pos_)
    {
        cube_pos = pos_;

        IP6d pose;
        std::vector<IV3d> rot{
            IV3d(0, 0, 0),
            IV3d(0, 0, IGN_DTOR(90)),
            IV3d(0, 0, IGN_DTOR(180)),
            IV3d(0, 0, IGN_DTOR(270)),
            IV3d(0, IGN_DTOR(90), 0),
            IV3d(0, IGN_DTOR(-90), 0)
        };

        for(int i=0; i<6; i++){
            pose.Set(pos_, rot[i]);
            cubes[i]->SetWorldPose(pose);
            cubes[i]->Update();
        }
    }

    IV3d GetPos() const {   return cube_pos;    }

    void GetCubeMap()
    {
        for(int i=0; i<6; i++){
            gzmsg << "rendering cube map " << i << std::endl;
            cubes[i]->Render(true);
            cubes[i]->PostRender();
            cubes[i]->SaveFrame("/tmp/cubemap_" + std::to_string(i) + ".png");
        }
    }

    static IV3d Project(const rendering::CameraPtr& cam, const IV3d& pt)
    {
        IV3d scr;
        auto pos = cam->OgreCamera()->getProjectionMatrix() *
                cam->OgreCamera()->getViewMatrix() *    //rendering::Conversions::Convert(pt);
                Ogre::Vector4(rendering::Conversions::Convert(pt));

        pos.x /= pos.w;
        pos.y /= pos.w;
        
        ignition::math::Vector2i screenPos;
        scr.X() = ((pos.x / 2.0) + 0.5) * cam->ViewportWidth();
        scr.Y() = (1.0 - ((pos.y / 2.0) + 0.5)) * cam->ViewportHeight();
        scr.Z() = pos.z;

        return scr;
    }

    IV3i GetRGB(const IV3d& pt)
    {
        IV3i rgb;
        
        for(int i=0; i<6; i++){
            auto scr = Project(cubes[i], pt);
            if(scr.X() >= 0 && int(scr.X()) < text_size && 
                scr.Y() >= 0 && int(scr.Y()) < text_size &&
                scr.Z() > 0.0)
            {
                int u = scr.X();
                int v = scr.Y();
                int idx = 3 * (v * text_size + u);
                const unsigned char* data = &(cubes[i]->ImageData()[idx]);

                rgb.X() = data[0];
                rgb.Y() = data[1];
                rgb.Z() = data[2];

                return rgb;
            }
        }
    }
    
    template <typename T>
    pcrgb_t::Ptr StainPC(const pc_ptr_<T>& pc)
    {
        pcrgb_t::Ptr pc_rgb(new pcrgb_t);
        pc_rgb->resize(pc->size());

        for(int i=0; i<pc->size(); i++)
        {
            auto& pt = pc->points[i];
            auto& prgb = pc_rgb->points[i];

            auto rgb = GetRGB(IV3d(pt.x, pt.y, pt.z));
            prgb.x = pt.x;
            prgb.y = pt.y;
            prgb.z = pt.z;
            prgb.r = rgb.X();
            prgb.g = rgb.Y();
            prgb.b = rgb.Z();
        }

        return pc_rgb;
    }

protected:

    rendering::ScenePtr scene;
    std::vector<rendering::CameraPtr> cubes;
    int text_size;

    IV3d cube_pos;

};

}

