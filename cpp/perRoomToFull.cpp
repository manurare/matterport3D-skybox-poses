//
// Created by manuel on 10/06/2021.
//

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"

#include <fstream>
#include <memory>
#include <string>
#include "openMVG/stl/split.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

std::map<int, int> read_mapping(std::string filename);

int main(int argc, char** argv) {
    CmdLine cmd;

    std::string perRoomPosesDir;

    cmd.add(make_option('p', perRoomPosesDir, "per_room_poses_dir"));

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch (const std::string &s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-p|--per_room_poses_dir]\n"
                  << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    if (!perRoomPosesDir.empty() && *perRoomPosesDir.rbegin() != '/')
        perRoomPosesDir += '/';

    // Configure an empty scene with Views and their corresponding cameras
    openMVG::sfm::SfM_Data sfm_data;
    openMVG::sfm::Views &views = sfm_data.views;
    openMVG::sfm::Intrinsics &intrinsics = sfm_data.intrinsics;
    openMVG::sfm::Poses &poses = sfm_data.poses;
    openMVG::sfm::Landmarks &structure = sfm_data.structure;

    std::vector<std::string> allData = stlplus::folder_files( perRoomPosesDir );
    std::vector<std::string> perRoomSfMdata, key_mapping_files;

    for (std::vector<std::string>::const_iterator allData_iter = allData.begin();
         allData_iter != allData.end(); ++allData_iter){
        if (allData_iter->find("key_mapping") != std::string::npos) {
            key_mapping_files.push_back(*allData_iter);
        }else{
            if(*allData_iter != "sfm_data.json") perRoomSfMdata.push_back(*allData_iter);
        }
    }

    std::sort(key_mapping_files.begin(), key_mapping_files.end());
    std::sort(perRoomSfMdata.begin(), perRoomSfMdata.end());

    if(key_mapping_files.size() != perRoomSfMdata.size()){
        std::cerr<<"Not same size"<<std::endl;
        return 1;
    }

    for (std::pair<std::vector<std::string>::const_iterator, std::vector<std::string>::const_iterator>
            i(perRoomSfMdata.begin(), key_mapping_files.begin()); i.first != perRoomSfMdata.end();
            ++i.first, ++i.second) {
        std::map<int, int> mapping = read_mapping(perRoomPosesDir + (*i.second));
        openMVG::sfm::SfM_Data roomSfMData;
        Load(roomSfMData, perRoomPosesDir + (*i.first), openMVG::sfm::ESfM_Data::ALL);

        if (i.first == perRoomSfMdata.begin()){
            sfm_data.s_root_path = roomSfMData.s_root_path; // WRONG Setup main image root_path
            intrinsics = roomSfMData.intrinsics; // Setup main image root_path
        }
        auto iter_view = roomSfMData.views.begin();
        for (; iter_view != roomSfMData.views.end(); ++iter_view) {
            View v(iter_view->second->s_Img_path, mapping[iter_view->second->id_view], iter_view->second->id_intrinsic,
                   mapping[iter_view->second->id_view], iter_view->second->ui_width, iter_view->second->ui_height);
            views[v.id_view] = std::make_shared<View>(v);
            if (roomSfMData.poses.find(iter_view->second->id_pose) != roomSfMData.poses.end())
                poses[v.id_pose] = roomSfMData.poses[iter_view->second->id_pose];


        }
    }
    // Store SfM_Data views & intrinsic data
    if (!Save(
            sfm_data,
            stlplus::create_filespec( perRoomPosesDir, "sfm_data.json" ).c_str(),
            ESfM_Data(ALL)))
    {
        return EXIT_FAILURE;
    }
}


std::map<int, int> read_mapping(std::string filename){
    std::map<int, int> mapping;
    std::ifstream myfile (filename);
    std::string line;
    std::vector<std::string> split_line;
    if (myfile.is_open())
    {
        std::vector<std::string> vec_str;
        while ( getline (myfile,line) ) {
            vec_str.clear();
            stl::split(line, ' ', vec_str);
            int key = std::atoi(vec_str[0].c_str());
            int value = std::atoi(vec_str[1].c_str());
            mapping.insert({key, value});
        }
    }
    return mapping;
}