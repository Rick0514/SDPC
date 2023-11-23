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

public:

    static vector<vector<double>> parsePath(string fn)
    {
        string tmp;    
        std::ifstream inf(fn);
        // first line
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
        vector<double> d(6);
        for(int i=0; i<6; i++){
            d[i] = sps[i](t);
        }
        ignition::math::Pose3d p(d[0], d[1], d[2], d[3], d[4], d[5]);
        return p;
    }

private:

    double total_time;
    vector<tk::spline> sps;
};

