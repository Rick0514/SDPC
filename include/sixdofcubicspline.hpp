#pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <ignition/math.hh>

#include "cubicspline.h"

using std::string;
using std::vector;

class SixDofCubicSpline
{
private:
    static constexpr double Deg2Rad = M_PI / 180.0;

    using IV3d = ignition::math::Vector3d;
    using IQd = ignition::math::Quaterniond;

    double total_time;
    vector<tk::spline> sps; // xyz, rpy

    double still_time;
    ignition::math::Pose3d still_pose;

public:

    static vector<vector<double>> parsePath(string fn)
    {
        string tmp;    
        std::ifstream inf(fn);
        // jump first two lines
        getline(inf, tmp);
        getline(inf, tmp);

        vector<vector<double>> data;
        vector<double> line;
        line.reserve(7);

        while(getline(inf, tmp))
        {
            tmp += ',';

            std::stringstream ss(tmp);
            string cc;
            while(getline(ss, cc, ',')){
                double d = stod(cc);
                line.push_back(d);
            }

            data.push_back(line);
            line.clear();
        }

        // print for debug
        // int nl = data.size();
        // for (size_t i = 0; i < nl; i++)
        // {
        //     for(auto&& e : data[i]) cout << e <<" ";
        //     cout << endl;
        // }

        return data;
    }

    SixDofCubicSpline(double total_time_, string fn) : total_time(total_time_)
    {
        auto data = parsePath(fn);
        int data_num = data.size();
        vector<double> t;
        for(auto& e : data)    t.push_back(e[0]);

        double time_scale = *std::max_element(t.begin(), t.end());
        for(auto& e : t)   e = (e / time_scale) * total_time;

        still_time = t[1];
        still_pose.Set(data[0][1], data[0][2], data[0][3], data[0][4] * Deg2Rad, data[0][5] * Deg2Rad, data[0][6] * Deg2Rad);
        t.erase(t.begin());
        data.erase(data.begin());

        // init sps
        for(int i=1; i<=6; i++){
            tk::spline sp;
            sp.set_boundary(tk::spline::bd_type::first_deriv, 0,
                tk::spline::bd_type::first_deriv, 0);
        
            vector<double> d;
            for(auto& e : data){
                if(i < 4)
                    d.push_back(e[i]);
                else
                    d.push_back(e[i] * Deg2Rad);
            }
            sp.set_points(t, d);
            sps.push_back(sp);
        }
    }

    ignition::math::Pose3d getPose(double t)
    {
        if(t < still_time)  return still_pose;

        vector<double> d(6);
        for(int i=0; i<6; i++){
            d[i] = sps[i](t);
        }
        ignition::math::Pose3d p(d[0], d[1], d[2], d[3], d[4], d[5]);
        return p;
    }

    IV3d getV(double t)
    {
        if(t < still_time)  return IV3d::Zero;

        vector<double> v(3);
        for(int i=0; i<3; i++){
            v[i] = sps[i].deriv(1, t);
        }
        return IV3d(v[0], v[1], v[2]);
    }

    IV3d getA(double t)
    {
        if(t < still_time)  return IV3d::Zero;

        vector<double> v(3);
        for(int i=0; i<3; i++){
            v[i] = sps[i].deriv(2, t);
        }
        return IV3d(v[0], v[1], v[2]);
    }

    // https://github.com/rpng/lips/blob/master/lips_matlab/matlab/functions/lips/euler2angular.m
    IV3d getOmega(double t)
    {
        if(t < still_time)  return IV3d::Zero;

        auto pose = getPose(t);

        double roll = pose.Rot().Roll();
        double pitch = pose.Rot().Pitch();
        double yaw = pose.Rot().Yaw();

        double dr = sps[3].deriv(1, t);
        double dp = sps[4].deriv(1, t);
        double dy = sps[5].deriv(1, t);

        // dot(w) = J * dot(rpy)
        ignition::math::Matrix3d J(cos(yaw)*cos(pitch), -sin(yaw), 0,
            sin(yaw)*cos(pitch), cos(yaw), 0,
            -sin(pitch), 0, 1
        );
        
        IV3d om_inG = J * IV3d(dr, dp, dy);
        return om_inG;
    }

    IV3d getAlpha(double t)
    {
        if(t < still_time)  return IV3d::Zero;

        auto pose = getPose(t);

        double roll = pose.Rot().Roll();
        double pitch = pose.Rot().Pitch();
        double yaw = pose.Rot().Yaw();

        auto cr = cos(roll);
        auto sr = sin(roll);
        auto cp = cos(pitch);
        auto sp = sin(pitch);
        auto cy = cos(yaw);
        auto sy = sin(yaw);
        
        double dr = sps[3].deriv(1, t);
        double dp = sps[4].deriv(1, t);
        double dy = sps[5].deriv(1, t);
        double ddr = sps[3].deriv(2, t);
        double ddp = sps[4].deriv(2, t);
        double ddy = sps[5].deriv(2, t);

        // dot(w) = J * dot(rpy)
        ignition::math::Matrix3d J(
            cy*cp, -sy, 0,
            sy*cp, cy, 0,
            -sp, 0, 1
        );

        // ddot(w) = dJ * dot(rpy) + J * ddot(rpy)
        ignition::math::Matrix3d dJ(
            -sy*cp*dy-sp*cy*dp, -cy*dy, 0,
            cy*cp*dy-sp*sy*dp, -sy*dy, 0,
            -cp*dp, 0, 0
        );
        
        IV3d al_inG = dJ * IV3d(ddr, ddp, ddy) + J * IV3d(dr, dp, dy);
        return al_inG;
    }
};

