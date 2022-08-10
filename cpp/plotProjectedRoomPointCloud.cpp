//
// Created by manuel on 14/06/2021.
//

#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "openMVG/cameras/Camera_Spherical.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include <opencv2/opencv.hpp>
#include "depth_io.hpp"
#include <math.h>

std::tuple<std::vector<openMVG::Vec2> , std::vector<openMVG::Vec3>> visiblePtsInFrame(openMVG::sfm::SfM_Data sfm_data, openMVG::IndexT idx);
std::tuple<float, std::vector<openMVG::Vec2>> reprojectionError(openMVG::sfm::SfM_Data, openMVG::IndexT, std::vector<openMVG::Vec2>, std::vector<openMVG::Vec3>);

int main(int argc, char** argv) {
    CmdLine cmd;

    std::string sfmDataDir;

    cmd.add(make_option('i', sfmDataDir, "sfm_data_dir"));

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch (const std::string &s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-i|--sfm_data_dir]\n"
                  << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::string sfm_data_file, outDir;
    outDir = stlplus::create_filespec(sfmDataDir, "GT");
    stlplus::folder_create(outDir);
    sfm_data_file = stlplus::create_filespec(sfmDataDir, "sfm_data.bin");
    openMVG::sfm::SfM_Data room_sfm_data;
    openMVG::sfm::Load(room_sfm_data, sfm_data_file, openMVG::sfm::ESfM_Data::ALL);

    auto iter_view = room_sfm_data.views.begin();
    for (; iter_view != room_sfm_data.views.end(); ++iter_view) {

        std::string img_name = iter_view->second->s_Img_path;
        std::string imgNameNoExt;
        size_t dot = img_name.find_last_of(".");
        if (dot != std::string::npos)
        {
            imgNameNoExt = img_name.substr(0, dot);
        }

        // If there is not element, pass
        if (room_sfm_data.poses.find(iter_view->second->id_pose) == room_sfm_data.poses.end()) continue;

        std::vector<openMVG::Vec2> vis_pixels, proj_pixels;
        std::vector<openMVG::Vec3> vis_worldPts;
        std::tie(vis_pixels, vis_worldPts) = visiblePtsInFrame(room_sfm_data, iter_view->second->id_view);

        //Project 3d world coords to cam coord
        std::vector<openMVG::Vec3> vis_camPts;
        for (int i = 0; i < vis_worldPts.size(); i++) {
            vis_camPts.push_back(room_sfm_data.poses.find(iter_view->second->id_pose)->second.operator()(vis_worldPts.at(i)));
        }

        float repr_error;
        std::tie(repr_error, proj_pixels) = reprojectionError(room_sfm_data, iter_view->second->id_view, vis_pixels, vis_camPts);

        cv::Mat rgb_img = cv::imread(stlplus::create_filespec(room_sfm_data.s_root_path, iter_view->second->s_Img_path), cv::IMREAD_COLOR);
        cv::Mat1f sparse_depth = cv::Mat1f::zeros(rgb_img.rows, rgb_img.cols);
        for (int i = 0; i < vis_pixels.size(); i++) {
            cv::Point tmp, projection;
            tmp.x = vis_pixels.at(i)[0];
            tmp.y = vis_pixels.at(i)[1];
            projection.x = proj_pixels.at(i)[0];
            projection.y = proj_pixels.at(i)[1];

            cv::circle(rgb_img, tmp, 3, cv::Scalar(0, 0, 255),
                       cv::FILLED, cv::LINE_8);
            cv::circle(rgb_img, projection, 3, cv::Scalar(255, 0, 0),
                       cv::FILLED, cv::LINE_8);

            float norm = sqrt(pow(vis_camPts.at(i)[0],2) + pow(vis_camPts.at(i)[1],2) +
                    pow(vis_camPts.at(i)[2],2));
            sparse_depth.at<float>(tmp) = norm;
        }
        std::string proj_point_cloud;
        std::string outfile = img_name.substr(0, img_name.find('.'));
        proj_point_cloud = outfile + std::string("_pc.png");
        outfile += std::string("_gt.dpt");
        cv::imwrite(stlplus::create_filespec(outDir, proj_point_cloud), rgb_img);
    }

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


