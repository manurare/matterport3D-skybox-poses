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


def get_uuids(conf_file):
    with open(conf_file) as f:
        content = f.readlines()
    uuid_imgs = [line.splitlines() for line in content if "scan" in line]
    uuids = np.array([line[0].split(' ')[1].split('_')[0] for line in uuid_imgs])
    return np.unique(uuids)


def parse_conf_file(conf_file, img_uuid):
    with open(conf_file) as f:
        content = f.readlines()
    uuid_imgs = [line.splitlines() for line in content if img_uuid in line]
    poses = [np.array(line[0].split(' ')[3:]).astype(np.float64).reshape(4, 4) for line in uuid_imgs]
    return np.stack(poses)


def write_params_to_file(out_file, img_name, idx, cam_center, rot):
    print(idx)
    with open(out_file, 'a') as params_file:
        params_file.write("{} {} {} {} {} {} {} {} {}\n".format(idx, img_name,
                                                                cam_center[0], cam_center[1], cam_center[2],
                                                                rot[0], rot[1], rot[2], rot[3]))


def upright_rotation(vertical_cam_axis, vertical_world_axis):
    v = np.cross(vertical_cam_axis, vertical_world_axis)
    s = np.linalg.norm(v)
    c = np.dot(vertical_cam_axis, vertical_world_axis)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    r = np.eye(3) + vx + (vx @ vx) * (1 - c) / (s ** 2)
    return r

pref = bpy.context.preferences.addons['cycles'].preferences
pref.compute_device_type = 'CUDA'
print('Devices:')
for i, device in enumerate(pref.get_devices()[0]): # [0] = cuda, [1] = OpenCL
    print('    {}. {} ({})'.format(i, device.name, device.type))
    device.use = True

# Read camera poses
scan_path = "/mnt/fast0/mra59/matterport/data/"
sequences = [os.path.basename(os.path.dirname(element)) for element in glob.glob(os.path.join(scan_path, "*", ""))]
sequences.sort()
for seq in sequences:
    clear_collections()

    if sys.platform == 'win32':
        poses_path = r"D:\PHD\matterport\data\v1\scans\{}\{}\matterport_camera_poses".format(seq, seq)
        conf_file_undistorted = r"D:\PHD\matterport\data\v1\scans\{}\{}\undistorted_camera_parameters\{}.conf".format(seq, seq, seq)
        mesh_basepath = r"D:\PHD\matterport\data\v1\scans\{}\{}\matterport_mesh".format(seq, seq)
        tmp = [name for name in os.listdir(mesh_basepath) if not os.path.isdir(name)]
        mesh_path = r"D:\PHD\matterport\data\v1\scans\{}\{}\matterport_mesh\{}\{}.obj".format(seq, seq, tmp[0], tmp[0])
        out_path = r"D:\PHD\matterport\data\v1\scans\{}\{}\rendered_textures".format(seq, seq)
    else:
        base_path = "/mnt/fast0/mra59/matterport/data/{}/".format(seq, seq)
        conf_file_undistorted = os.path.join(base_path, "undistorted_camera_parameters/{}.conf".format(seq))
        mesh_basepath = os.path.join(base_path, "matterport_mesh")
        tmp = [name for name in os.listdir(mesh_basepath) if not os.path.isdir(name)]
        mesh_path = os.path.join(mesh_basepath, tmp[0], "{}.obj".format(tmp[0]))
        out_path = os.path.join(base_path, "rendered_textures")
        # out_path = "/media/manuel/Elements/PHD/matterport/data/v1/scans/{}/{}/debug".format(seq, seq)

    # Cycles Render Engine
    bpy.data.scenes["Scene"].render.engine = "CYCLES"
    bpy.data.scenes["Scene"].cycles.samples = 1
    bpy.data.scenes["Scene"].render.image_settings.color_mode = 'RGB'

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

    pose_uuids = get_uuids(conf_file_undistorted)

    # Camera object
    cam = bpy.data.objects['Camera']
    bpy.context.scene.camera = cam
    cam.rotation_mode = "QUATERNION"

    # Matterport camera cs is y down, x right, z look
    # Blender camera cs is y up, x right, -z look
    wb2wo = R.from_euler('XYZ', [-np.pi * 0.5, 0, 0])    # world blender cs to world openMVG cs
    cb2co = R.from_euler('XYZ', [np.pi, 0, 0])    # camera blender cs to camera openMVG cs
    skybox_facing = R.from_euler('XYZ', [0, 0, np.pi/2])    # Save approximated skybox camera param

    groups = bpy.data.node_groups
    # mesh_obj = bpy.data.objects[mesh_filename]
    for key in bpy.data.objects.keys():
        if "Camera" in key or "camera" in key:
            continue
        for idx, slot in enumerate(bpy.data.objects[key].material_slots):
            material = slot.material
            bsdf = [node for node in material.node_tree.nodes if node.name == "Principled BSDF"]
            if bsdf:
                material.node_tree.nodes.remove(bsdf[0])
            material.node_tree.links.new(material.node_tree.nodes['Image Texture'].outputs['Color'],
                                         material.node_tree.nodes['Material Output'].inputs['Surface'])

    canonical_displacement = np.array([0.5, 0, 0])
    up = np.array([0, 0, 0.5])
    down = np.array([0, 0, -0.5])
    displaced_middle = np.array([R.from_euler('XYZ', [0, 0, rot], degrees=True).as_matrix() @ canonical_displacement
                                 for rot in np.arange(0, 360, 60)])
    displaced_up = np.array([R.from_euler('XYZ', [0, 0, rot], degrees=True).as_matrix() @ (canonical_displacement + up)
                             for rot in np.arange(0, 360, 60)])
    displaced_down = np.array([R.from_euler('XYZ', [0, 0, rot], degrees=True).as_matrix() @ (canonical_displacement + down)
                               for rot in np.arange(0, 360, 60)])

    displaced_centers = np.stack((displaced_up, displaced_middle, displaced_down)).reshape(-1, 3)

    # Header of camera parameters file
    os.makedirs(out_path, exist_ok=True)
    openMVG_GT_params = os.path.join(out_path, 'openMVG_GT_cam_poses.txt')
    blender_GT_params = os.path.join(out_path, 'blender_GT_cam_poses.txt')
    if os.path.isfile(openMVG_GT_params):
        os.remove(openMVG_GT_params)
    if os.path.isfile(blender_GT_params):
        os.remove(blender_GT_params)

    with open(openMVG_GT_params, 'w') as params_file:
        params_file.write("#index filename trans_wc.x trans_wc.y trans_wc.z quat_wc.x quat_wc.y quat_wc.z quat_wc.w\n")
    with open(blender_GT_params, 'w') as params_file:
        params_file.write("#index filename trans_wc.x trans_wc.y trans_wc.z quat_wc.x quat_wc.y quat_wc.z quat_wc.w\n")

    cont = 0
    for idx, uuid in enumerate(pose_uuids):
        print(uuid)
        # if "0f7e0af0cb3b4c2abf62bba2fd955702" not in uuid:
        #     continue
        poses = parse_conf_file(conf_file_undistorted, uuid)

        # og camera center
        C = np.mean(poses[:, :3, 3], axis=0)

        for idx_displacement in range(0, displaced_centers.shape[0]):
            # the 8th index is element (2,3) in 3x6 camera array
            rot_cb = R.from_matrix(poses[8, 0:3, 0:3])
            vertical_cam_axis = rot_cb.as_matrix() @ np.array([0, 1, 0])
            vertical_world_axis = np.array([0, 0, 1])
            r = upright_rotation(vertical_cam_axis, vertical_world_axis)
            rot_cb = R.from_matrix(r @ poses[8, 0:3, 0:3])   # upright
            rot_quat_cb = rot_cb.as_quat()

            cam.rotation_quaternion[0] = rot_quat_cb[3]
            cam.rotation_quaternion[1] = rot_quat_cb[0]
            cam.rotation_quaternion[2] = rot_quat_cb[1]
            cam.rotation_quaternion[3] = rot_quat_cb[2]

            new_center = C + displaced_centers[idx_displacement]
            cam.location.x = new_center[0]
            cam.location.y = new_center[1]
            cam.location.z = new_center[2]

            # camera_rot_co2wo = wb2wo.as_matrix() @ rot_cb.as_matrix().T @ cb2co.as_matrix().T
            camera_rot_co2wo = rot_cb.as_matrix() @ cb2co.as_matrix().T
            camera_rot_wo2co = np.linalg.inv(camera_rot_co2wo)

            rot_quat_co2wo = R.from_matrix(camera_rot_co2wo).as_quat()
            rot_quat_wo2co = R.from_matrix(camera_rot_wo2co).as_quat()

            # Render RGB + depth
            bpy.data.scenes["Scene"].render.filepath = os.path.join(out_path, uuid + "_{:02}_rgb".format(idx_displacement))

            write_params_to_file(openMVG_GT_params, os.path.basename(bpy.data.scenes["Scene"].render.filepath),
                                 cont, new_center, rot_quat_wo2co)
            write_params_to_file(blender_GT_params, os.path.basename(bpy.data.scenes["Scene"].render.filepath),
                                 cont, new_center, rot_quat_cb)
            cont += 1

            if os.path.isfile(bpy.data.scenes["Scene"].render.filepath + bpy.data.scenes["Scene"].render.file_extension):
                continue
            bpy.ops.render.render(write_still=True)

        # skybox cam param
        rot_cb = skybox_facing.as_matrix() @ poses[8, 0:3, 0:3]
        vertical_cam_axis = rot_cb @ np.array([0, 1, 0])
        vertical_world_axis = np.array([0, 0, 1])
        r = upright_rotation(vertical_cam_axis, vertical_world_axis)
        rot_cb = r @ rot_cb

        # camera_rot_co2wo = wb2wo.as_matrix() @ rot_cb.T @ cb2co.as_matrix().T
        camera_rot_co2wo = rot_cb @ cb2co.as_matrix().T
        camera_rot_wo2co = np.linalg.inv(camera_rot_co2wo)

        rot_quat_co2wo = R.from_matrix(camera_rot_co2wo).as_quat()
        rot_quat_wo2co = R.from_matrix(camera_rot_wo2co).as_quat()

        write_params_to_file(openMVG_GT_params, uuid + "_{:02}_rgb".format(displaced_centers.shape[0]),
                             cont, C, rot_quat_wo2co)
        write_params_to_file(blender_GT_params, uuid + "_{:02}_rgb".format(displaced_centers.shape[0]),
                             cont, C, R.from_matrix(rot_cb).as_quat())
        cont += 1

    print(pose_uuids.shape[0])
