#!/usr/bin/python
#! -*- encoding: utf-8 -*-

# This file is part of OpenMVG (Open Multiple View Geometry) C++ library.

# Python implementation of the bash script written by Romuald Perrot
# Created by @vins31
# Modified by Pierre Moulon
#
# this script is for easy use of OpenMVG
#
# usage : python openmvg.py image_dir output_dir
#
# image_dir is the input directory where images are located
# output_dir is where the project must be saved
#
# if output_dir is not present script will create it
#

# Indicate the openMVG binary directory
OPENMVG_SFM_BIN = "/home/manuel/libraries/openMVG/bin"

# Indicate the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/manuel/Desktop/PHD/code/openMVG/src/software/SfM" + "/../../openMVG/exif/sensor_width_database"

import os
import subprocess
import sys
import glob

if len(sys.argv) < 3:
    print ("Usage %s image_dir output_dir" % sys.argv[0])
    sys.exit(1)

input_dir = sys.argv[1]
output_dir = sys.argv[2]
mixedPoses = True
matches_dir = os.path.join(output_dir, "matches")
reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

print ("Using input dir  : ", input_dir)
print ("      output_dir : ", output_dir)

imgs = glob.glob(input_dir + "/*.png")
imgs.sort()
# size = np.array([cv2.imread(img, cv2.IMREAD_UNCHANGED).shape for img in imgs])
# height, width = np.mean(size[:, 0], axis=0), np.mean(size[:, 1], axis=0)
# f = 1.2*np.maximum(height, width)
f = 4608.0

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

if not os.path.isfile(reconstruction_dir+"/sfm_data.bin") or not os.path.isfile(reconstruction_dir+"/cloud_and_poses.ply"):

    print ("1. Intrinsics analysis")
    # pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  "-i", input_dir, "-o", matches_dir, "-d", camera_file_params, "-f", str(f)] )
    pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  "-i", input_dir, "-o", matches_dir, "-c", str(7), "-f", str(1)] )
    pIntrisics.wait()

    print ("2. Compute features")
    pFeatures = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT", "-p", "HIGH"] )
    pFeatures.wait()

    print ("3. Compute matches")
    pMatches = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-g", "a"] )
    pMatches.wait()

    # Create the reconstruction if not present
    if not os.path.exists(reconstruction_dir):
        os.mkdir(reconstruction_dir)

    if not mixedPoses:
        print ("4. Do Sequential/Incremental reconstruction")
        # pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"),  "-i", matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir] )
        pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"),  "-i",
                                     matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir,
                                     "-a", os.path.basename(imgs[0]), "-b", os.path.basename(imgs[3])] )

    else:

        print ("4. Do Sequential/Incremental reconstruction with known poses")
        pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM2"),  "-i",
                                     input_dir+os.sep+os.pardir+"/GTPoses_sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir,
                                     "-S", "EXISTING_POSE"] )

    pRecons.wait()
    print ("7. Convert camera parameters bin file to json")
    # pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", reconstruction_dir+"/sfm_data.json", "-I", "-E"] )
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", reconstruction_dir+"/sfm_data.json"] )
    pRecons.wait()


else:
    print ("7. Convert camera parameters bin file to json")
    # pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", reconstruction_dir+"/sfm_data.json", "-I", "-E"] )
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", reconstruction_dir+"/sfm_data.json"] )
    pRecons.wait()
    # random_cam = np.random.randint(0, len(imgs)-1, 1)
    # cam_params = json.load(open(reconstruction_dir+"/sfm_data.json"))
    # print("hola")


