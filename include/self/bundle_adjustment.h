//
// Created by tonychen on 18-11-27.
//

#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include "common.h"
namespace Slam2D {
    class ReprojectionError {
    public:
        ReprojectionError(Slam2D::Point2d observed_uv)
                : uv_(observed_uv) {}
        template<typename T>
        bool operator()(const T *const cam_pos,
                        const T *const keypoint,
                        const T *const camera,
                        T *residuals) const;
        template <typename T>
        static inline bool predictUV(const T* const cam_pos, const T *const keypoint,
                            const T *const camera, T* predict);
        static ceres::CostFunction *create(const Slam2D::Point2d observed_uv);
    private:
        Slam2D::Point2d uv_;
    };

    // Bundle adjustment problem
    class BundleAdjustment {
    public:
        typedef std::shared_ptr<BundleAdjustment> Ptr;
        static Ptr create(){ return Ptr(new BundleAdjustment());}
        BundleAdjustment() = default;
        ~BundleAdjustment() = default;

        void addObservation(const string frame_id, const string keypoint_id, const Slam2D::Point2d observed_uv);
        void compile(ceres::LossFunction* loss=NULL);
        void solve(bool stdout_progress=true, ceres::LinearSolverType type=ceres::DENSE_SCHUR);
        string fullReport();
        string briefReport();

        void save(string model_file);
        void writePlyFile(string ply_file);
        friend ostream& operator<< (ostream &os, const BundleAdjustment& ba);

        std::vector<Slam2D::Point2d> getObserveError() const { return error_;};

    private:
        std::vector<double> parameters_;
        ceres::Problem problem_;
        ceres::Solver::Summary summary_;
        void calcCameraCenter(int frame_id, double *cam_center);

        std::vector<string> frame_id_;      // all frame ids list
        std::vector<string> keypoint_id_;   // all keypoint ids list

        std::vector<string> frame_uid_;   //unique id(erase repeat elements from 'frame_id_')
        std::vector<string> keypoint_uid_; //unique keypoint id

        std::vector<Slam2D::Point2d> observed_uv_; //observed value u list

        std::vector<Slam2D::Point2d> error_; //problem error

        inline double *getFramePtr(string frame_id);
        inline double *getKeypointPtr(string keypoint_id);
        inline double *getFramePtr(int observe_id);
        inline double *getKeypointPtr(int observe_id);
        inline double *getCameraPtr();
    };
}


#endif //BUNDLE_ADJUSTMENT_H
