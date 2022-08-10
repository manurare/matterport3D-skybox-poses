//
// Created by manuel on 09/04/2021.
//

#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "openMVG/cameras/Camera_Spherical.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include <opencv2/opencv.hpp>
#include "depth_io.hpp"
#include <math.h>

std::tuple<std::vector<openMVG::Vec2> , std::vector<openMVG::Vec3>> visiblePtsInFrame(openMVG::sfm::SfM_Data sfm_data, openMVG::IndexT idx);
std::tuple<float, std::vector<openMVG::Vec2>> reprojectionError(openMVG::sfm::SfM_Data, openMVG::IndexT, std::vector<openMVG::Vec2>, std::vector<openMVG::Vec3>);

int main(int argc, char** argv) {
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");


    std::string json_file, ply_file, img_folder, out_dir, bin_file;
    if (argc < 2){
        std::cerr << "Provide dataset path" << std::endl;
        exit(0);
    } else{
        json_file = argv[1] + std::string("/Global_rec/reconstruction_sequential/sfm_data.json");
        bin_file = argv[1] + std::string("/Global_rec/reconstruction_sequential/sfm_data.bin");
        ply_file =  argv[1] + std::string("/Global_rec/reconstruction_sequential/cloud_and_poses.ply");
        img_folder = argv[1] + std::string("/Input/");
        out_dir = argv[1] + std::string("/GT/");

        // Set to 1 argv[2] to process per room configuration
        if (argv[2] != NULL){
            json_file = argv[1] + std::string("/per_room_sfm/sfm_data.json");
        }
    }
    openMVG::sfm::SfM_Data sfm_data;
    if (argv[2] != NULL){
        openMVG::sfm::Load(sfm_data, json_file, openMVG::sfm::ESfM_Data::ALL);
    }else{
        openMVG::sfm::Load(sfm_data, bin_file, openMVG::sfm::ESfM_Data::ALL);
    }

    int total_cam = sfm_data.views.size();
    std::vector<float> repr_errors;
    std::vector<std::vector<std::string>> extrinsics;

    auto iter_view = sfm_data.views.begin();
    for (; iter_view != sfm_data.views.end(); ++iter_view) {
//        openMVG::IndexT cam_idx = rand() % total_cam;
        int cam_idx = iter_view->second->id_view;
        std::vector<std::string> extr;
        std::string img_name = iter_view->second->s_Img_path;
        std::cout<<img_name<<std::endl;

        std::string imgNameNoExt;
        size_t dot = img_name.find_last_of(".");
        if (dot != std::string::npos)
        {
            imgNameNoExt = img_name.substr(0, dot);
        }

        // If there is not element, pass
        if (sfm_data.poses.find(iter_view->second->id_pose) == sfm_data.poses.end()) continue;

        Eigen::Quaterniond q(sfm_data.poses.find(iter_view->second->id_pose)->second.rotation());
        std::cout<<sfm_data.poses.find(iter_view->second->id_pose)->second.rotation().format(CleanFmt)<<std::endl;

        extr.push_back(std::to_string(cam_idx));
        extr.push_back(imgNameNoExt);
//        extr.push_back(std::to_string((-sfm_data.poses.at(cam_idx).rotation().transpose()*sfm_data.poses.at(cam_idx).center())[0]));
//        extr.push_back(std::to_string((-sfm_data.poses.at(cam_idx).rotation().transpose()*sfm_data.poses.at(cam_idx).center())[1]));
//        extr.push_back(std::to_string((-sfm_data.poses.at(cam_idx).rotation().transpose()*sfm_data.poses.at(cam_idx).center())[2]));
        extr.push_back(std::to_string(sfm_data.poses.find(iter_view->second->id_pose)->second.center()[0]));
        extr.push_back(std::to_string(sfm_data.poses.find(iter_view->second->id_pose)->second.center()[1]));
        extr.push_back(std::to_string(sfm_data.poses.find(iter_view->second->id_pose)->second.center()[2]));
        extr.push_back(std::to_string(q.coeffs().x()));
        extr.push_back(std::to_string(q.coeffs().y()));
        extr.push_back(std::to_string(q.coeffs().z()));
        extr.push_back(std::to_string(q.coeffs().w()));

        extrinsics.push_back(extr);

        std::vector<openMVG::Vec2> vis_pixels, proj_pixels;
        std::vector<openMVG::Vec3> vis_worldPts;
        std::tie(vis_pixels, vis_worldPts) = visiblePtsInFrame(sfm_data, cam_idx);

        //Move 3d world coords to cam coord
        std::vector<openMVG::Vec3> vis_camPts;
        for (int i = 0; i < vis_worldPts.size(); i++) {
            vis_camPts.push_back(sfm_data.poses.find(iter_view->second->id_pose)->second.operator()(vis_worldPts.at(i)));
        }

        float repr_error;
        std::tie(repr_error, proj_pixels) = reprojectionError(sfm_data, cam_idx, vis_pixels, vis_camPts);
        repr_errors.push_back(repr_error);
//        cv::Mat rgb_img = cv::imread(img_folder + img_name, cv::IMREAD_COLOR);
//        cv::Mat1f sparse_depth = cv::Mat1f::zeros(rgb_img.rows, rgb_img.cols);
//        for (int i = 0; i < vis_pixels.size(); i++) {
//            cv::Point tmp, projection;
//            tmp.x = vis_pixels.at(i)[0];
//            tmp.y = vis_pixels.at(i)[1];
//            projection.x = proj_pixels.at(i)[0];
//            projection.y = proj_pixels.at(i)[1];
//
//            cv::circle(rgb_img, tmp, 3, cv::Scalar(0, 0, 255),
//                       cv::FILLED, cv::LINE_8);
//            cv::circle(rgb_img, projection, 3, cv::Scalar(255, 0, 0),
//                       cv::FILLED, cv::LINE_8);
//
//            float norm = sqrt(pow(vis_camPts.at(i)[0],2) + pow(vis_camPts.at(i)[1],2) +
//                    pow(vis_camPts.at(i)[2],2));
//            sparse_depth.at<float>(tmp) = norm;
//        }
//        std::string proj_point_cloud;
//        std::string outfile = img_name.substr(0, img_name.find('.'));
//        proj_point_cloud = outfile + std::string("_pc.png");
//        outfile += std::string("_gt.dpt");
////        writeSintelDptFile(out_dir + outfile, sparse_depth);
//        cv::imwrite(out_dir + proj_point_cloud, rgb_img);
    }
    std::string output_filename = argv[1] + std::string("/reprojection_errors.txt");
    std::ofstream output_file(output_filename.c_str());
    for (const auto &e : repr_errors) output_file << e << "\n";
    output_file.close();


    std::string output_extr = argv[1] + std::string("/camera_params_estimation.txt");
    std::ofstream output_extr_f(output_extr.c_str());
    output_extr_f << "#index filename trans_wc.x trans_wc.y trans_wc.z quat_wc.x quat_wc.y quat_wc.z quat_wc.w \n";
    for (const auto &e : extrinsics) {
        for (int j=0; j<e.size(); j++){
            output_extr_f << e[j] << " ";
        }
        output_extr_f << "\n";
    }
    output_extr_f.close();
    return 0;
}

std::tuple<std::vector<openMVG::Vec2> , std::vector<openMVG::Vec3>> visiblePtsInFrame(openMVG::sfm::SfM_Data sfm_data, openMVG::IndexT idx){
    std::vector<openMVG::Vec2> pix2d;
    std::vector<openMVG::Vec3> world_3d;
    for (auto const& x : sfm_data.GetLandmarks()){
        if ( x.second.obs.find(idx) != x.second.obs.end() ) {
            pix2d.push_back(x.second.obs.find(idx)->second.x);
            world_3d.push_back(x.second.X);
        }
    }

    assert(pix2d.size() == world_3d.size());
    openMVG::Mat2X vis_pixels(2, pix2d.size());
    openMVG::Mat3X vis_worldPoints(3, pix2d.size());
    for (int i = 0; i < pix2d.size(); i++) {
        vis_pixels(0, i) = pix2d.at(i)[0];
        vis_pixels(1, i) = pix2d.at(i)[1];

        vis_worldPoints(0, i) = world_3d.at(i)[0];
        vis_worldPoints(1, i) = world_3d.at(i)[1];
        vis_worldPoints(2, i) = world_3d.at(i)[2];
    }
    return {pix2d, world_3d};
}


std::tuple<float, std::vector<openMVG::Vec2>> reprojectionError(openMVG::sfm::SfM_Data sfm_data, openMVG::IndexT idx,
                        std::vector<openMVG::Vec2> vis_pixels, std::vector<openMVG::Vec3> vis_camPts){
    std::vector<float> err_per_pt;
    openMVG::Vec2 err;
    std::vector<openMVG::Vec2> repr_pix;
    for (int i=0; i<vis_pixels.size(); i++){
        err = sfm_data.intrinsics.at(0)->residual(vis_camPts.at(i), vis_pixels.at(i));
        err_per_pt.push_back(sqrt(pow(err[0],2) + pow(err[1],2)));

        repr_pix.push_back(sfm_data.intrinsics.at(0)->project(vis_camPts.at(i)));
    }
    float mean_err = std::accumulate(err_per_pt.begin(), err_per_pt.end(), 0.0) / err_per_pt.size();
//    std::cout << mean_err << std::endl;
    return {mean_err, repr_pix};
}
