//
// Created by manuel on 19/05/2021.
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

std::map<std::string, Pose3> loadKnownExtrinsics(std::string extrinsicsFile, std::string skybox_idx);
std::string getSkyboxIdx(std::string extrinsicsFile);

int main(int argc, char** argv) {
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    CmdLine cmd;

    std::string sImageDir,
            sfileDatabase = "",
            sOutputDir,
            extrinsicsFile,
            sKmatrix;
    int i_User_camera_model = openMVG::cameras::CAMERA_SPHERICAL;
    bool b_Group_camera_model = true;
    double focal_pixels = 1.0;

    cmd.add( make_option('i', sImageDir, "imageDirectory") );
    cmd.add( make_option('c', i_User_camera_model, "camera_model") );
    cmd.add( make_option('o', sOutputDir, "outputDirectory") );
    cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );
    cmd.add( make_option('e', extrinsicsFile, "extrinsics_file") );
    cmd.add( make_option('f', focal_pixels, "focal") );

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch (const std::string& s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-i|--imageDirectory]\n"
                  << "[-e|--extrinsics_file]\n"
                  << "[-f|--focal] (pixels)\n"
                  << "[-c|--camera_model] Camera model type:\n"
                  << "\t" << static_cast<int>(openMVG::cameras::PINHOLE_CAMERA) << ": Pinhole\n"
                  << "\t" << static_cast<int>(openMVG::cameras::PINHOLE_CAMERA_RADIAL1) << ": Pinhole radial 1\n"
                  << "\t" << static_cast<int>(openMVG::cameras::PINHOLE_CAMERA_RADIAL3) << ": Pinhole radial 3 (default)\n"
                  << "\t" << static_cast<int>(openMVG::cameras::PINHOLE_CAMERA_BROWN) << ": Pinhole brown 2\n"
                  << "\t" << static_cast<int>(openMVG::cameras::PINHOLE_CAMERA_FISHEYE) << ": Pinhole with a simple Fish-eye distortion\n"
                  << "\t" << static_cast<int>(openMVG::cameras::CAMERA_SPHERICAL) << ": Spherical camera\n"
                  << "[-g|--group_camera_model]\n"
                  << "\t 0-> each view have it's own camera intrinsic parameters,\n"
                  << "\t 1-> (default) view can share some camera intrinsic parameters\n"
                  << "[-o|--outputDirectory]\n"
                  << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    sOutputDir = stlplus::create_filespec(sImageDir, "../");
    std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
    std::sort(vec_image.begin(), vec_image.end());

    std::string skybox_idx = getSkyboxIdx(extrinsicsFile);
    std::map<std::string, Pose3> knownExtrinsics = loadKnownExtrinsics(extrinsicsFile, skybox_idx);

    // Configure an empty scene with Views and their corresponding cameras
    openMVG::sfm::SfM_Data sfm_data;
    sfm_data.s_root_path = sImageDir; // Setup main image root_path
    openMVG::sfm::Views & views = sfm_data.views;
    openMVG::sfm::Intrinsics & intrinsics = sfm_data.intrinsics;
    openMVG::sfm::Poses & poses = sfm_data.poses;

    double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;

    C_Progress_display my_progress_bar( vec_image.size(),
                                        std::cout, "\n- Image listing -\n" );
    std::ostringstream error_report_stream;
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
          iter_image != vec_image.end();
          ++iter_image, ++my_progress_bar ) {
        std::string imgNameNoExt;
        size_t dot = iter_image->find_last_of(".");
        if (dot != std::string::npos) {
            imgNameNoExt = iter_image->substr(0, dot);
        }

        // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
        width = height = ppx = ppy = focal = -1.0;

        const std::string sImageFilename = stlplus::create_filespec(sImageDir, *iter_image);
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        // Test if the image format is supported:
        if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown) {
            error_report_stream
                    << sImFilenamePart << ": Unkown image file format." << "\n";
            continue; // image cannot be opened
        }

        if (sImFilenamePart.find("mask.png") != std::string::npos
            || sImFilenamePart.find("_mask.png") != std::string::npos) {
            error_report_stream
                    << sImFilenamePart << " is a mask image" << "\n";
            continue;
        }

        ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue; // image cannot be read

        width = imgHeader.width;
        height = imgHeader.height;
        ppx = width / 2.0;
        ppy = height / 2.0;
        focal = focal_pixels;

        // Build intrinsic parameter related to the view
        std::shared_ptr<IntrinsicBase> intrinsic;

        if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0) {
            // Create the desired camera type
            switch (i_User_camera_model) {
                case PINHOLE_CAMERA:
                    intrinsic = std::make_shared<Pinhole_Intrinsic>
                            (width, height, focal, ppx, ppy);
                    break;
                case PINHOLE_CAMERA_RADIAL1:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
                            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_RADIAL3:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_BROWN:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_FISHEYE:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case CAMERA_SPHERICAL:
                    intrinsic = std::make_shared<Intrinsic_Spherical>
                            (width, height);
                    break;
                default:
                    std::cerr << "Error: unknown camera model: " << (int) i_User_camera_model << std::endl;
                    return EXIT_FAILURE;
            }
        }

        // Build the view corresponding to the image
        View v(*iter_image, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr) {
            //Since the view have invalid intrinsic data
            // (export the view, with an invalid intrinsic field value)
            v.id_intrinsic = UndefinedIndexT;
        } else {
            // Add the defined intrinsic to the sfm_container
            intrinsics[v.id_intrinsic] = intrinsic;
        }

        // Add the view to the sfm_container
        views[v.id_view] = std::make_shared<View>(v);

        //    Assign pose here
        if (knownExtrinsics.find(imgNameNoExt) != knownExtrinsics.end())
            poses[v.id_pose] = knownExtrinsics[imgNameNoExt];
//            std::cout << "check" << std::endl;
//            Eigen::Quaterniond check_quat(poses[v.id_pose].rotation());
//            Eigen::Quaterniond check_other(0.5, 0.5, 0.5, 0.5);
//            std::cout << check_quat.x() << std::endl;
//            std::cout << check_quat.y() << std::endl;
//            std::cout << check_quat.z() << std::endl;
//            std::cout << check_quat.w() << std::endl;
//            std::cout << poses[v.id_pose].rotation().format(CleanFmt) << std::endl;
//            std::cout << check_quat.toRotationMatrix().format(CleanFmt) << std::endl;
//            std::cout << check_other.toRotationMatrix().format(CleanFmt) << std::endl;
    }

    // Display saved warning & error messages if any.
    if (!error_report_stream.str().empty())
    {
        std::cerr
                << "\nWarning & Error messages:" << std::endl
                << error_report_stream.str() << std::endl;
    }

    // Group camera that share common properties if desired (leads to more faster & stable BA).
    if (b_Group_camera_model)
    {
        GroupSharedIntrinsics(sfm_data);
    }

    // Store SfM_Data views & intrinsic data
    if (!Save(
            sfm_data,
            stlplus::create_filespec( sOutputDir, "GTPoses_sfm_data.json" ).c_str(),
            ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS)))
    {
        return EXIT_FAILURE;
    }

    std::cout << std::endl
              << "SfMInit_ImageListing report:\n"
              << "listed #File(s): " << vec_image.size() << "\n"
              << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
              << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;

    return EXIT_SUCCESS;
}


std::map<std::string, Pose3> loadKnownExtrinsics(std::string extrinsicsFile, std::string skybox_idx){
    std::map<std::string, Pose3> knownPoses;
    std::string line;
    std::ifstream myfile (extrinsicsFile);
    std::vector<std::string> split_line;

    Eigen::Quaternion<double> rot_quat;
    Mat3 rot;
    Vec3 center;
    std::string field;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            split_line.clear();
            if ((line[0] == '#') || (line.find("_"+skybox_idx) != std::string::npos)) continue;
            std::istringstream s(line);
            while (getline(s, field,' ')){
                split_line.push_back(field);
            }
            if (split_line.size() < 9) continue;

            std::cout<<split_line[1]<<std::endl;
            center = Eigen::Vector3d(stod(split_line[2]), stod(split_line[3]), stod(split_line[4]));
            rot_quat = Eigen::Quaternion<double>(stod(split_line[8]), stod(split_line[5]), stod(split_line[6]), stod(split_line[7]));
            knownPoses.insert(std::pair<std::string, Pose3>(split_line[1], Pose3(rot_quat.toRotationMatrix(), center)));
        }
        myfile.close();
    }

    else std::cout << "Unable to open file";

    return knownPoses;
}

std::string getSkyboxIdx(std::string extrinsicsFile){
    std::ifstream myfile (extrinsicsFile);
    std::string line;
    std::vector<int> idxs;
    std::vector<std::string> split_line;
    std::string field;
    std::string delim = "_";
    if (myfile.is_open()) {
        while ( getline (myfile,line) )
        {
            split_line.clear();
            std::istringstream s(line);
            while (getline(s, field,' ')){
                split_line.push_back(field);
            }

            if (split_line.size() < 9) continue;

            auto start = 0U;
            auto end = split_line[1].find(delim);
            int cont = 0;
            while (end != std::string::npos)
            {
                if (cont==1){
                    idxs.push_back(std::stoi(split_line[1].substr(start, end - start)));
                    break;
                }
                start = end + std::string(delim).length();
                end = split_line[1].find(delim, start);
                cont++;
            }
        }
    }
    int idx = *std::max_element(std::begin(idxs), std::end(idxs));
    std::string dest;
    if (idx < 10){
        dest = std::string( 1, '0').append( std::to_string(idx));
    }else{
        dest = std::to_string(idx);
    }
    return dest;
}