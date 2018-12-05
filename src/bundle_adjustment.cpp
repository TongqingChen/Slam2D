//
// Created by tonychen on 18-11-27.
//
#include <iomanip>
#include <fstream>
#include <self/bundle_adjustment.h>

#include "../include/self/bundle_adjustment.h"
#include "../include/self/model.h"
namespace Slam2D
{
#define CAM_POS_DIM     2
#define KEY_POINT_DIM   2
#define CAM_PARA_DIM    4 //dimensions for each input

    template <typename T>
    bool ReprojectionError::operator()(
            const T *const cam_pos,  //observe points, each has 2 values x, y(camera position)
            const T *const keypoint, //keypoints, each has 2 values X, Y
            const T *const camera,   //camera, all points and keypoints share it.
            T *residuals) const
    {
        T predict[2];
        predictUV(cam_pos, keypoint, camera, predict);
        residuals[0] = predict[0] - T(uv_.x);
        residuals[1] = predict[1] - T(uv_.y);
        return true;
    }
    ceres::CostFunction * ReprojectionError::create(const Slam2D::Point2d observed_uv) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, CAM_POS_DIM, KEY_POINT_DIM, CAM_PARA_DIM>(
                new ReprojectionError(observed_uv)));
    }

    template<typename T>
    bool ReprojectionError::predictUV(const T *const cam_pos, const T *const keypoint, const T *const camera, T* predict)
    {
        //camera[0, 1] //intra-para
        // -- f, distortion(only considering 2-order radial)
        //camera[2, 3] //outer para
        // -- camera rotation theta and camera Z(related to keypoints)

        T p[3];
        Slam2D::Point2<T> xy(keypoint[0], keypoint[1]);
        Slam2D::Point2<T> new_xy = xy.rotate(camera[2]);

        // translation
        p[0] = new_xy.x +  cam_pos[0];
        p[1] = new_xy.y + cam_pos[1];
        p[2] = camera[3]; //each observation has the same dZ;

        // formula: u = f*x/z, v = f*y/z

        // Compute the center of distortion.
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // Apply 2nd-order radial distortion.
        T r2 = xp*xp + yp*yp;
        T distortion = T(1.0) + r2  * camera[1];

        // Compute final projected point position.
        predict[0] = camera[0] * distortion * xp;
        predict[1] = camera[0] * distortion * yp;
        return true;
    }


    //insert observations to problem
    void BundleAdjustment::addObservation(const string frame_id, const string keypoint_id, const Slam2D::Point2d observed_uv)
    {
        frame_id_.push_back(frame_id);
        keypoint_id_.push_back(keypoint_id);
        if(std::find(frame_uid_.begin(), frame_uid_.end(), frame_id)==frame_uid_.end()){
            frame_uid_.push_back(frame_id);
        }
        if(std::find(keypoint_uid_.begin(), keypoint_uid_.end(), keypoint_id)==keypoint_uid_.end()){
            keypoint_uid_.push_back(keypoint_id);
        }
        observed_uv_.push_back(observed_uv);
    }

    // compile the bundle adjustment problem
    void BundleAdjustment::compile(ceres::LossFunction* loss)
    {
        //生成随机数作为初始值
        srand(time(0));
        int len = frame_uid_.size()*CAM_POS_DIM + keypoint_uid_.size()*KEY_POINT_DIM + CAM_PARA_DIM;
        for(int i = 0; i<len; i++) {
            parameters_.push_back(rand()%100 / 101.0);
        }
        for(size_t i=0; i<observed_uv_.size(); i++)
        {
            problem_.AddResidualBlock(
                    ReprojectionError::create(observed_uv_[i]),
                    loss, /* loss function, default squared loss */
                    getFramePtr(i),
                    getKeypointPtr(i),
                    getCameraPtr()
                    );
        }
    }

    // solve the ba problem
    void BundleAdjustment::solve(bool stdout_progress, ceres::LinearSolverType type)
    {
        ceres::Solver::Options options;
        options.max_num_iterations = 5000;
        options.num_threads = 16;
        options.linear_solver_type = type;
        options.minimizer_progress_to_stdout = stdout_progress;
        ceres::Solve(options, &problem_, &summary_);

        //calculate error
        for(int i=0; i<observed_uv_.size(); i++)
        {
            double *kp = getKeypointPtr(i);
            double *cp = getFramePtr(i);
            double *camera = getCameraPtr();

            double predict[2];
            ReprojectionError::predictUV(cp, kp, camera, predict);

            // The error is the difference between the predicted and observed position.
            error_.push_back(Slam2D::Point2d(predict[0], predict[1]) - observed_uv_[i]);
        }
    }

    string BundleAdjustment::briefReport()
    {
        return this->summary_.BriefReport();
    }

    string BundleAdjustment::fullReport()
    {
        return this->summary_.FullReport();
    }

    void BundleAdjustment::writePlyFile(string ply_file)
    {
        std::ofstream fout(ply_file.c_str());

        fout<< "ply"
        << '\n' << "format ascii 1.0"
        << '\n' << "element vertex " << frame_uid_.size() + keypoint_uid_.size()
        << '\n' << "property float x"
        << '\n' << "property float y"
        << '\n' << "property float z"
        << '\n' << "property uchar red"
        << '\n' << "property uchar green"
        << '\n' << "property uchar blue"
        << '\n' << "end_header\n";

        // camera pose, optical center/camera center
        double cam_center[3]; //c = -R'*t
        for(int i = 0; i < frame_uid_.size(); ++i){
            calcCameraCenter(i, cam_center);
            fout << cam_center[0] << ' '
            << cam_center[1] << ' '
            << cam_center[2]
            << " 0 255 0\n";
        }

        // Export the kepoint (i.e. 3D Points) as white points.
        for(int i = 0; i < keypoint_uid_.size(); ++i){
            fout << getKeypointPtr(keypoint_uid_[i])[0] << ' '
            << getKeypointPtr(keypoint_uid_[i])[1] << ' '
            << 0
            << " 255 255 255\n";
        }
        fout.close();
    }
    //overide operator<< to support std::cout;
    ostream& operator<< (ostream &os, const BundleAdjustment& ba)
    {
        os  << "--------------------------" << std::endl;
        os  << std::setiosflags(ios::left) << std::setw(12) << "Item" << std::resetiosflags(ios::left) // 用完之后清除
            << std::setiosflags(ios::right) << std::setw(10) << "Number" << std::endl;
        os  << "--------------------------" << std::endl;
        os  << std::setiosflags(ios::left) << setw(12) << "Frame" << resetiosflags(ios::left)
            << std::setiosflags(ios::right) << std::setw(10) << ba.frame_uid_.size() << std::endl;
        os  << std::setiosflags(ios::left) << setw(12) << "Keypoint" << resetiosflags(ios::left)
            << std::setiosflags(ios::right) << std::setw(10) << ba.keypoint_uid_.size() << std::endl;
        os  << std::setiosflags(ios::left) << setw(12) << "Observation" << resetiosflags(ios::left)
            << std::setiosflags(ios::right) << std::setw(10) << ba.observed_uv_.size() << std::endl;
        os  << "--------------------------" << std::endl;
    }
    void BundleAdjustment::save(string model_file) {
        Slam2D::Model model;
        model.setCamera(getCameraPtr());
        for(auto kp:keypoint_uid_)
        {
            model.insertMap(kp, getKeypointPtr(kp));
        }
        model.save(model_file);
    }



    //------------------------------ private functions ---------------------
    void BundleAdjustment::calcCameraCenter(int frame_id, double *cam_center)
    {   //c = -R'*t
        Slam2D::Point2d xy(getFramePtr(frame_uid_[frame_id]));
        Slam2D::Point2d new_xy = xy.rotate(-getCameraPtr()[2]);
        cam_center[0] = -new_xy.x;
        cam_center[1] = -new_xy.y;
        cam_center[2] = -getCameraPtr()[3];
    }

    double *BundleAdjustment::getFramePtr(string frame_id)
    {
        int id = std::distance(
                frame_uid_.begin(),
                std::find(frame_uid_.begin(), frame_uid_.end(), frame_id)
        );
        return &parameters_[0] + id*CAM_POS_DIM;
    }

    double *BundleAdjustment::getKeypointPtr(string keypoint_id)
    {
        int id = std::distance(
                keypoint_uid_.begin(),
                std::find(keypoint_uid_.begin(), keypoint_uid_.end(), keypoint_id)
                );
        return &parameters_[0] + frame_uid_.size()*CAM_POS_DIM + id*KEY_POINT_DIM;
    }

    double *BundleAdjustment::getCameraPtr()
    {
        return &parameters_[0] + frame_uid_.size()*CAM_POS_DIM + keypoint_uid_.size()*KEY_POINT_DIM;
    }

    double *BundleAdjustment::getFramePtr(int observe_id) {
        return getFramePtr(frame_id_[observe_id]);
    }

    double *BundleAdjustment::getKeypointPtr(int observe_id) {
        return getKeypointPtr(keypoint_id_[observe_id]);
    }

}