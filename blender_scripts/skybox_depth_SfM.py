import numpy as np
import glob
import os
from scipy.spatial.transform import Rotation as R
import sys
import bpy

def clear_collections():
    for obj in [o for o in bpy.data.collections['Collection'].objects]:
        # Delete the object
        bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.orphans_purge()


scan_path = "/mnt/fast0/mra59/matterport/data/"
sequences = [os.path.basename(os.path.dirname(element)) for element in glob.glob(os.path.join(scan_path, "*", ""))]
sequences.sort()

for seq in sequences:
    clear_collections()

    base_path = os.path.join(scan_path,"{}".format(seq))
    skybox_params_file = os.path.join(base_path, "skybox_camera_parameters.txt")
    mesh_basepath = os.path.join(base_path, "matterport_mesh")
    tmp = [name for name in os.listdir(mesh_basepath) if not os.path.isdir(name)]
    mesh_path = os.path.join(mesh_basepath, tmp[0], "{}.obj".format(tmp[0]))
    out_path = os.path.join(base_path, "depth")
    os.makedirs(out_path, exist_ok=True)

    # Cycles Render Engine
    bpy.data.scenes["Scene"].render.engine = "CYCLES"

    # GPU as device
    bpy.data.scenes["Scene"].cycles.device = "GPU"

    if len(bpy.data.cameras) == 0:
        camera_data = bpy.data.cameras.new(name="Camera")
        camera_object = bpy.data.objects.new("Camera", camera_data)
        bpy.context.scene.collection.objects.link(camera_object)

    # Set camera to equirectangular
    bpy.data.cameras["Camera"].type = "PANO"
    bpy.data.cameras["Camera"].cycles.panorama_type = "EQUIRECTANGULAR"

    # Image resolution
    bpy.context.scene.render.resolution_y = 1024
    bpy.context.scene.render.resolution_x = bpy.context.scene.render.resolution_y * 2

    # Remove default light and cube
    if "Cube" in bpy.data.objects.keys():
        bpy.data.objects['Cube'].select_set(True)
        bpy.ops.object.delete()

    if "Light" in bpy.data.objects.keys():
        bpy.data.objects['Light'].select_set(True)
        bpy.ops.object.delete()

    # Import mesh if it hasn't been previously loaded
    mesh_filename = os.path.basename(mesh_path).split(".")[0]
    if mesh_filename not in bpy.data.objects.keys():
        bpy.ops.import_scene.obj(filepath=mesh_path, axis_up='Z', axis_forward='Y')

    # Camera object
    cam = bpy.data.objects['Camera']
    bpy.context.scene.camera = cam
    cam.rotation_mode = "QUATERNION"

    # Parse skybox camera params
    with open(skybox_params_file, 'r') as f:
        content = f.readlines()

    content = [line.splitlines() for line in content]
    params_skybox = [np.array(line[0].split(' ')) for line in content if line[0][0] != "#"]
    for idx, skybox_cam in enumerate(params_skybox):
        print(float(skybox_cam[0]))

        uuid = skybox_cam[1].split('_')[0]
        C = np.array([float(skybox_cam[2]), float(skybox_cam[3]), float(skybox_cam[4])])

        rot_quat = np.array([float(skybox_cam[5]), float(skybox_cam[6]), float(skybox_cam[7]), float(skybox_cam[8])])

        cam.rotation_quaternion[0] = rot_quat[3]
        cam.rotation_quaternion[1] = rot_quat[0]
        cam.rotation_quaternion[2] = rot_quat[1]
        cam.rotation_quaternion[3] = rot_quat[2]

        cam.location.x = C[0]
        cam.location.y = C[1]
        cam.location.z = C[2]

        # Render RGB + depth
        print("RENDERING")

        bpy.context.scene.node_tree.nodes["File Output.001"].file_slots[0].path = uuid + "_depth"
        bpy.context.scene.node_tree.nodes["File Output.001"].base_path = out_path

        outfile_wrong = os.path.join(bpy.context.scene.node_tree.nodes["File Output.001"].base_path,
                                     bpy.context.scene.node_tree.nodes["File Output.001"].file_slots[
                                         0].path + "0000.exr".format(idx))
        outfile_correct = os.path.join(bpy.context.scene.node_tree.nodes["File Output.001"].base_path,
                                       bpy.context.scene.node_tree.nodes["File Output.001"].file_slots[
                                           0].path + ".exr".format(idx))

        bpy.data.scenes["Scene"].render.filepath = os.path.join(out_path, "remove")

        if os.path.isfile(outfile_correct):
            continue
        bpy.ops.render.render(write_still=True)
        os.rename(outfile_wrong, outfile_correct)
























































































































