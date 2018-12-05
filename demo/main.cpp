//
// Created by tonychen on 18-11-27.
//
#include <fstream>
#include "../include/self/bundle_adjustment.h"
#include "../include/self/model.h"

int main()
{
    string observe_file = "../dataset/keypoints_std_new.txt";
    string out_ply_file = "../output/demo.ply";
    string model_file = "../output/model.txt";
    Slam2D::Point2d image_size(2048, 2048);

    ifstream fin(observe_file);
    if(!fin)
    {
        std::cerr<<"find no observe_file:"<<observe_file<<std::endl;
        return -1;
    }
    // 1. construct a bundle adjustment problem
    auto ba = Slam2D::BundleAdjustment::create();

    // 2. add observations to the problem
    std::string frame_id, keypoint_id;
    Slam2D::Point2d observed_uv;

    std::vector<string> keypoint_ids;   // service for predict
    std::vector<Slam2D::Point2d> observations;

    while(fin>>frame_id>>keypoint_id>>observed_uv.x>>observed_uv.y)
    {
        ba->addObservation(
                frame_id, keypoint_id,
                observed_uv-image_size*0.5 // to simplify the model(remove cx and cy)
                );
        keypoint_ids.push_back(keypoint_id);
        observations.push_back(observed_uv-image_size*0.5);
    }
    fin.close();

    std::cout<<*ba<<std::endl;

    // 3. compile the problem
    ba->compile();

    // 4. solve the problem
    ba->solve();

    // 5. print out solve result
    std::cout<<ba->fullReport()<<std::endl;

    for(auto m:ba->getObserveError())
    {
        std::cout<<"delta_x="<<m.x<<", delta_y="<<m.y<<std::endl;
    }
    // 6. save ply file
    ba->writePlyFile(out_ply_file);

    // 7. save_model
    ba->save(model_file);

    //-----------------------predict-----------------------
    auto model = Slam2D::Model();
    // 1. load model
    model.load(model_file);
    // 2. predict
    Slam2D::Point2d xy;
    for(int i=0; i<keypoint_ids.size(); i++)
    {
        model.predictCameraPos(keypoint_ids[i], observations[i], xy);
        std::cout<<"predict x="<<xy.x<<", predict y="<<xy.y<<endl;
    }
}