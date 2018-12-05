//
// Created by tonychen on 18-11-29.
//
#include "../include/self/model.h"
namespace Slam2D {
    void Model::load(const string model_file) {
        ifstream fin(model_file);
        if (!fin) {
            std::cerr << "model file not exsit: " << model_file << endl;
            return;
        }
        fin >> camera[0] >> camera[1] >> camera[2] >> camera[3];
        Slam2D::Point2d xy;
        string keypoint;
        while (fin >> keypoint >> xy.x >> xy.y) {
            map_.insert(make_pair(keypoint, xy));
        }
        fin.close();
    }

    void Model::save(const string model_file) {
        ofstream fout(model_file);
        fout << camera[0] <<' '<< camera[1]<<' '<< camera[2]<<' '<< camera[3] << endl;
        for (auto m:map_) {
            fout << m.first << ' '
            <<m.second.x << ' '
            <<m.second.y << endl;
        }
        fout.close();
    }

    bool Model::predictCameraPos(const string kp,
                                          const Slam2D::Point2d uv,
                                          Slam2D::Point2d &xy) {
        if (map_.find(kp) == map_.end()) {
            std::cerr << "find no key point: " << kp << std::endl;
            return false;
        }
        /*formula
         ---1---
         f*[1+distortion*(xp^2+yp^2)]*xp = u
         f*[1+distortion*(xp^2+yp^2)]*yp = v
         ---2---
         f*[1+distortion*(xp^2+(v/u)^2*xp^2]*xp = u
         ---3---
         xp^3+p*xp+q = 0
         where p=1/[distortion*(1+(v/u)^2)], q = -u/[f*distortion*(1+(v/u)^2)]
         ---4---
         xp = [-q/2+((q/2)^2+(p/3)^2)^0.5]^(1/3) + [-q/2-((q/2)^2+(p/3)^2)^0.5]^(1/3)
         yp = v*xp/u
         ---5---
         xp = (X+t[0])/Z
         yp = (Y+t[1])/Z
        */
        Slam2D::Point2d XY = map_[kp];

        auto c = XY.rotate(camera[2]);

        /* //below is method for considering distortion
         Slam2D::Point2d p;

         if (fabs(uv.x) < std::numeric_limits<double>::epsilon() &&
             fabs(uv.y) < std::numeric_limits<double>::epsilon()) {
             p = Slam2D::Point2d::zeros();
         } else if (fabs(uv.x) < std::numeric_limits<double>::epsilon()) {
             p.x = 0;
             temp = uv.y*sqrt(camera[1])*0.5/camera[0];
             p.y = pow(temp+sqrt(temp*temp+1.0/27), 1.0/3) + pow(temp-sqrt(temp*temp+1.0/27), 1.0/3);
             p.y /= sqrt(camera[1]);

          //   p_over_3 = 1.0 / (3 * camera[1]);
           //  q_over_2 = 0.5 * uv.y / (camera[0] * camera[1]);
            // p.y = pow((-q_over_2 + sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3) +
            //       pow((-q_over_2 - sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3);
         } else if (fabs(uv.y) < std::numeric_limits<double>::epsilon()) {
             p.y = 0;
             temp = uv.x*sqrt(camera[1])*0.5/camera[0];
             p.x = pow(temp+sqrt(temp*temp+1.0/27), 1.0/3) + pow(temp-sqrt(temp*temp+1.0/27), 1.0/3);
             p.x /= sqrt(camera[1]);
 //            p_over_3 = 1.0 / (3 * camera[1]);
 //            q_over_2 = 0.5 * uv.x / (camera[0] * camera[1]);
 //            p.x = pow((-q_over_2 + sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3) +
 //                  pow((-q_over_2 - sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3);
         } else {
             temp = uv.x*sqrt(camera[1])*0.5/camera[0];
             temp2 = temp + sqrt(temp*temp+1.0/27);
             if(temp2<0) {
                 temp3 = -pow(-temp2, 1.0 / 3);
             } else            {
                 temp3 = pow(temp2, 1.0/3);
             }
             temp2 = temp - sqrt(temp*temp+1.0/27);
             if(temp2<0) {
                 temp4 = -pow(-temp2, 1.0 / 3);
             } else            {
                 temp4 = pow(temp2, 1.0/3);
             }

             p.x = temp3+temp4;

             p.x /= sqrt(camera[1]);
 //            p_over_3 = 1.0 / (3 * camera[1]);
 //            q_over_2 = 0.5 * uv.x / (camera[0] * camera[1]);
 //            p.x = pow((-q_over_2 + sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3) +
 //                  pow((-q_over_2 - sqrt((pow(q_over_2, 2.0) + pow(p_over_3, 3.0)))), 1.0 / 3);
             p.y = p.x * uv.y / uv.x;
             xy = c - p * camera[3];
         }
          */

        xy = uv * (camera[3]/camera[0]);
        return true;
    }

    void Model::insertMap(const string key_point, const Slam2D::Point2d xy) {
        if (map_.find(key_point) == map_.end()) {
            map_.insert(make_pair(key_point, xy));
        } else {
            map_[key_point] = xy;
        }
    }

    void Model::setCamera(const double focal, const double distortion,
                                   const double theta, const double z) {
        camera[0] = focal;
        camera[1] = distortion;
        camera[2] = theta;
        camera[3] = z;
    }

    void Model::setCamera(const double *camera_info) {
        for(int i=0; i<4; i++) {
            camera[i] = camera_info[i];
        }
    }

    Model::Model() {
        map_.clear();
    }

    Model::~Model() {
        map_.clear();
    }
}