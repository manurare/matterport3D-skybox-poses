CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(sparse_gt_depth)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# openMVG
set(OpenMVG_DIR "/mnt/fast0/mra59/libraries/openMVG/share/openMVG/cmake")

find_package(OpenMVG REQUIRED)

# opencv
set(OpenCV_DIR "/mnt/fast0/mra59/libraries/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED )

add_executable(convertFormat convertFormat.cpp depth_io.cpp)

target_include_directories(convertFormat PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(convertFormat
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        ${OpenCV_LIBS}
        )


# openEXR
set(OpenEXR_DIR "/mnt/fast0/mra59/libraries/openEXR/lib/cmake/OpenEXR")
set(Imath_DIR "/mnt/fast0/mra59/libraries/openEXR/lib/cmake/Imath")
find_package(OpenEXR REQUIRED )

add_executable(depth_converter depth_map_convertor.cpp)
target_include_directories(depth_converter PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(depth_converter PRIVATE /home/manuel/libraries/openEXR/include /home/manuel/libraries/openEXR/include/OpenEXR /home/manuel/libraries/openEXR/include/Imath)

target_link_libraries(depth_converter
        PRIVATE
        ${OpenCV_LIBS}
        OpenEXR::OpenEXR
        )


add_executable(sfmDataFromKnownPoses sfmDataFromKnownPoses.cpp)

target_link_libraries(sfmDataFromKnownPoses
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_system
        )

add_executable(GTPerRoom GTPerRoom.cpp)

target_link_libraries(GTPerRoom
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_system
        )


add_executable(perRoomToFull perRoomToFull.cpp)

target_link_libraries(perRoomToFull
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_system
        )

add_executable(preserveUUID preserveUUID.cpp)

target_link_libraries(preserveUUID
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_system
        )

add_executable(plotProjectedRoomPointCloud plotProjectedRoomPointCloud.cpp)

target_link_libraries(plotProjectedRoomPointCloud
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        ${OpenCV_LIBS}
        )
