//
// Created by manuel on 11/06/2021.
//

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/cameras/cameras.hpp"
#include "openMVG/image/image_io.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;

int main(int argc, char** argv) {

    CmdLine cmd;

    std::string jsonMatches_dir,
            roomUuid;

    cmd.add(make_option('j', jsonMatches_dir, "json_matches_dir"));
    cmd.add(make_option('u', roomUuid, "room_uuid"));

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch (const std::string &s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-j|--json_matches_dir]\n"
                  << "[-u|--room_uuid]\n"
                  << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::string jsonMatches;
    jsonMatches = stlplus::create_filespec(jsonMatches_dir, "sfm_data.json");
    openMVG::sfm::SfM_Data all_matches;
    openMVG::sfm::Load(all_matches, jsonMatches, openMVG::sfm::ESfM_Data::ALL);

    // Configure an empty scene with Views and their corresponding cameras
    openMVG::sfm::SfM_Data uuid_matches;
    uuid_matches.s_root_path = all_matches.s_root_path; // Setup main image root_path
    openMVG::sfm::Views &views = uuid_matches.views;
    openMVG::sfm::Intrinsics &intrinsics = uuid_matches.intrinsics;
    openMVG::sfm::Poses &poses = uuid_matches.poses;

    intrinsics = all_matches.intrinsics;
    uuid_matches.s_root_path = stlplus::create_filespec(all_matches.s_root_path, +"../");

    for (auto iter_view = all_matches.views.begin(); iter_view != all_matches.views.end(); ++iter_view) {
        if (iter_view->second->s_Img_path.find(roomUuid) == std::string::npos) continue;
        View v(iter_view->second->s_Img_path, iter_view->second->id_view, iter_view->second->id_intrinsic,
               iter_view->second->id_pose, iter_view->second->ui_width, iter_view->second->ui_height);
        views[v.id_view] = std::make_shared<View>(v);

    }

    // Store SfM_Data views & intrinsic data
    if (!Save(
            uuid_matches,
            jsonMatches.c_str(),
            ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS)))
    {
        return EXIT_FAILURE;
    }
}