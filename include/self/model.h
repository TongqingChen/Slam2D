//
// Created by tonychen on 18-11-29.
//

#ifndef MODEL_H
#define MODEL_H

#include "common.h"

namespace Slam2D {
    class Model {
    public:
        Model();
        ~Model();

        void load(const string model_file);
        void save(const string model_file);
        bool predictCameraPos(const string key_point, const Slam2D::Point2d uv, Slam2D::Point2d& xy);

        void insertMap(const string key_point, const Slam2D::Point2d xy);
        void setCamera(double focal, double distortion, double theta, double z);
        void setCamera(const double* camera);

    private:
        std::map<string, Slam2D::Point2d> map_;
        double camera[4];
    };
}


#endif //SLAMAOI_MODEL_H
