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
import shutil

# Change these paths to your own
OPENMVG_SFM_BIN = "/mnt/fast0/mra59/libraries/openMVG/bin"
utils_bin = "/mnt/fast0/mra59/matterport/processing/misc/cpp/build"
scan_path = "/mnt/fast0/mra59/matterport/data/"
CAMERA_SENSOR_WIDTH_DIRECTORY = "/mnt/fast0/mra59/openMVG/src/software/SfM" + "/../../openMVG/exif/sensor_width_database" # Indicate the openMVG camera sensor width directory

import os
import subprocess
import sys
import glob
import numpy as np
import json
from shutil import copyfile
from shutil import move


def view_key_mapping(json_file, uuid):
    with open(json_file) as f:
        json_data = json.load(f)
    values = np.stack([element["key"] for element in json_data["views"] if uuid in element['value']['ptr_wrapper']['data']['filename']])
    keys = values - np.min(values)

    mapping = np.hstack((keys[..., None], values[..., None])).astype(np.int32)
    dir_path = os.path.dirname(json_file)
    np.savetxt(os.path.join(dir_path, "key_mapping_{}.txt".format(uuid)), mapping, fmt='%i')


sequences = [os.path.basename(os.path.dirname(element)) for element in glob.glob(os.path.join(scan_path, "*", ""))]
sequences.sort()
mixedPoses = True
for seq in sequences:
    input_imgs = []
    skybox_imgs = []
    renderings = glob.glob(os.path.join(scan_path, seq, "rendered_textures/*.png"))
    for filename in renderings:
        if "_18_rgb" in os.path.basename(filename):
            skybox_imgs.append(filename)
        else:
            input_imgs.append(filename)

    #if (len(input_imgs)//18) != len(skybox_imgs):
    #    continue
    txt_files = glob.glob(os.path.join(scan_path, seq, "rendered_textures/*.txt"))
    if txt_files:
        for txt_file in txt_files:
            shutil.move(txt_file, os.path.join(os.path.dirname(txt_file), os.pardir, os.path.basename(txt_file)))
    
    input_dir = os.path.join(scan_path, seq, "rendered_textures")
    output_dir = os.path.join(scan_path, seq, "Global_rec")
    camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

    print ("Using input dir  : ", input_dir)
    print ("      output_dir : ", output_dir)

    imgs = glob.glob(input_dir + "/*.png")
    imgs.sort()

    uuids = np.unique(np.array([os.path.basename(img_file).split('_')[0] for img_file in imgs]))

    per_room_sfm_folder = os.path.join(input_dir, os.pardir, "per_room_sfm")

    if os.path.isdir(per_room_sfm_folder):
        shutil.rmtree(per_room_sfm_folder)

    os.makedirs(per_room_sfm_folder, exist_ok=True)

    os.makedirs(output_dir, exist_ok=True)

    # Convert GT cam poses to openMVG json format
    if not os.path.isfile(os.path.join(scan_path, seq,"GTPoses_sfm_data.json")):
        pGTopenMVG = subprocess.Popen([os.path.join(utils_bin, "sfmDataFromKnownPoses"),
            "-i", input_dir, "-e", os.path.join(scan_path, seq,"openMVG_GT_cam_poses.txt")])
        pGTopenMVG.wait()

    # uuids = uuids[0:10]
    for idx, uuid in enumerate(uuids):
        # if "0f7e0af0cb3b4c2abf62bba2fd955702" not in uuid:
        #     continue
        print("{:04}/{:04}".format(idx+1, uuids.shape[0]))
        temp_room_imgs_folder = os.path.join(input_dir, "tmp_room_imgs_{}".format(uuid))
        output_dir_room = os.path.join(output_dir, "global_rec_{}".format(uuid))

        if not os.path.exists(output_dir_room):
            os.mkdir(output_dir_room)

        reconstruction_dir = os.path.join(output_dir_room, "reconstruction_sequential")
        matches_dir = os.path.join(output_dir_room, "matches")

        if not os.path.exists(matches_dir):
            os.mkdir(matches_dir)

        rec_sfm_data_filename_bin = os.path.join(reconstruction_dir, "sfm_data.bin")
        rec_sfm_data_filename_json = os.path.join(reconstruction_dir, "sfm_data_{}.json".format(uuid))
        matches_sfm_data_filename = os.path.join(matches_dir, "sfm_data.json")
        cloud_and_poses_filename = os.path.join(reconstruction_dir, "cloud_and_poses.ply")

        if not os.path.exists(temp_room_imgs_folder):
            os.makedirs(temp_room_imgs_folder, exist_ok=True)

        for img in imgs:
            if uuid in img:
                shutil.copy(img, temp_room_imgs_folder)

        if not os.path.isfile(rec_sfm_data_filename_bin) or not os.path.isfile(cloud_and_poses_filename):

            print ("1. Intrinsics analysis")
            pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  "-i", input_dir, "-o", matches_dir, "-c", str(7), "-f", str(1)] )
            pIntrisics.wait()

            # Create room sfm_data
            print("2. Discard all views from matches/sfm_data.json which do not belong to the current uuid")
            pDiscard = subprocess.Popen([os.path.join(utils_bin, "preserveUUID"),
                                         "-j", matches_dir, "-u", uuid])
            pDiscard.wait()
            view_key_mapping(matches_dir+"/sfm_data.json", uuid)

            print ("1. Intrinsics analysis")
            pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  "-i", temp_room_imgs_folder, "-o", matches_dir, "-c", str(7), "-f", str(1)] )
            pIntrisics.wait()

            print ("3. Compute features")
            pFeatures = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT", "-p", "ULTRA", "-u", str(1)] )
            pFeatures.wait()

            print ("4. Compute matches")
            pMatches = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-g", "a"] )
            pMatches.wait()

            # Create the reconstruction if not present
            if not os.path.exists(reconstruction_dir):
                os.mkdir(reconstruction_dir)

            # Create room sfm_data
            print("5. Extract room poses from the whole set of poses")
            pPerRoom = subprocess.Popen([os.path.join(utils_bin, "GTPerRoom"),
                                         "-p", os.path.join(input_dir, os.pardir), "-u", uuid])
            pPerRoom.wait()

            if not mixedPoses:
                print("6. Do Sequential/Incremental reconstruction")
                pRecons = subprocess.Popen([os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"), "-i",
                                            matches_dir + "/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir,
                                            "-a", os.path.basename(imgs[0]), "-b", os.path.basename(imgs[3])])

            else:

                print ("6. Do Sequential/Incremental reconstruction with known poses")
                pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM2"),  "-i",
                                             input_dir+os.sep+os.pardir+"/perRoom_sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir,
                                             "-S", "EXISTING_POSE"] )

            pRecons.wait()

            if os.path.isfile(reconstruction_dir+"/sfm_data.bin"):
                print ("7. Convert camera parameters bin file to json")
                pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", rec_sfm_data_filename_bin, "-o", rec_sfm_data_filename_json])
                pRecons.wait()
            else:
                shutil.rmtree(temp_room_imgs_folder)
                continue

        else:
            print ("7. Convert camera parameters bin file to json")
            pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", rec_sfm_data_filename_bin, "-o", rec_sfm_data_filename_json] )
            pRecons.wait()

        shutil.copy2(rec_sfm_data_filename_json, os.path.join(per_room_sfm_folder, os.path.basename(rec_sfm_data_filename_json)))
        shutil.copy2(os.path.join(matches_dir, "key_mapping_{}.txt".format(uuid)), os.path.join(per_room_sfm_folder, "key_mapping_{}.txt".format(uuid)))
        shutil.rmtree(temp_room_imgs_folder)


    # Assemble per room SfM to common file
    pRoomToFull = subprocess.Popen( [os.path.join(utils_bin, "perRoomToFull"),  "-p",  per_room_sfm_folder] )
    pRoomToFull.wait()

    # Change to procrustes format
    pToProcrustesFormat = subprocess.Popen( [os.path.join(utils_bin, "convertFormat"), os.path.join(scan_path, seq), "1"] )
    pToProcrustesFormat.wait()

