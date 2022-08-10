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

    std::string posesDir,
            roomUuid;

    cmd.add(make_option('p', posesDir, "full_poses_dir"));
    cmd.add(make_option('u', roomUuid, "room_uuid"));

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch (const std::string &s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-p|--full_poses_file]\n"
                  << "[-u|--room_uuid]\n"
                  << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::string fullPosesFile;
    fullPosesFile = stlplus::create_filespec(posesDir, "GTPoses_sfm_data.json");
    openMVG::sfm::SfM_Data room_sfm_data;
    openMVG::sfm::Load(room_sfm_data, fullPosesFile, openMVG::sfm::ESfM_Data::ALL);

    // Configure an empty scene with Views and their corresponding cameras
    openMVG::sfm::SfM_Data sfm_data;
    sfm_data.s_root_path = stlplus::create_filespec(posesDir, "Input/"); // Setup main image root_path
//    sfm_data.s_root_path = room_sfm_data.s_root_path; // Setup main image root_path
    openMVG::sfm::Views &views = sfm_data.views;
    openMVG::sfm::Intrinsics &intrinsics = sfm_data.intrinsics;
    openMVG::sfm::Poses &poses = sfm_data.poses;

    intrinsics = room_sfm_data.intrinsics;

    auto iter_view = room_sfm_data.views.begin();
    for (; iter_view != room_sfm_data.views.end(); ++iter_view) {

        if (iter_view->second->s_Img_path.find(roomUuid) == std::string::npos) continue;
        View v(iter_view->second->s_Img_path, views.size(), iter_view->second->id_intrinsic, views.size(), iter_view->second->ui_width, iter_view->second->ui_height);
        views[v.id_view] = std::make_shared<View>(v);
        if (room_sfm_data.poses.find(iter_view->first) == room_sfm_data.poses.end()) continue;
        else poses[v.id_view] = room_sfm_data.poses[iter_view->first];
    }

    // Store SfM_Data views & intrinsic data
    if (!Save(
            sfm_data,
            stlplus::create_filespec( posesDir, "perRoom_sfm_data.json" ).c_str(),
            ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS)))
    {
        return EXIT_FAILURE;
    }
}