import os.path
import os
import glob

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
import csv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


def visualize(GT, estimation):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(GT[:, 0], GT[:, 1], GT[:, 2], 'r*')
    ax.plot(estimation[:, 0], estimation[:, 1], estimation[:, 2], 'b^')
    ax.legend(['GT', 'SfM'])
    plt.show()


def get_skybox_idx(file_path):
    idxs = []
    with open(str(file_path), mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ', quotechar='|')
        line_count = 0
        for line_nb, row in enumerate(csv_reader):
            if line_nb <= 0:
                continue
            idxs.append(int(float(row[1].split("_")[1])))

    skybox_idx = "{:02}".format(np.max(np.array(idxs)))
    return skybox_idx


def per_room_poses(poses):
    room_split = {}
    for key, value in poses.items():
        key = key.split("_")[0]
        room_split.setdefault(key, []).append(value)

    for key, value in room_split.items():
        room_split[key] = np.stack(value)

    return room_split


def load_camposes(file_path, discard_idx="-1"):
    openvslam_result = {}
    with open(str(file_path), mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ', quotechar='|')
        line_count = 0
        for line_nb, row in enumerate(csv_reader):
            if line_nb <= 0:
                continue
            frame_idx = int(float(row[0]))

            if discard_idx != -1:
                if "_"+discard_idx in row[1]:
                    continue
            # change to euler angle, x y z w
            data = []
            data = data + [float(row[2]), float(row[3]), float(row[4])]
            rotation = R.from_quat([float(row[5]), float(row[6]), float(row[7]), float(row[8])])
            data = data + list(rotation.as_quat())

            openvslam_result[row[1]] = np.array(data)

    return openvslam_result


def procrustes_matlab(X, Y, scaling=True, reflection='best'):
    """
    A port of MATLAB's `procrustes` function to Numpy.

    Procrustes analysis determines a linear transformation (translation,
    reflection, orthogonal rotation and scaling) of the points in Y to best
    conform them to the points in matrix X, using the sum of squared errors
    as the goodness of fit criterion.

        d, Z, [tform] = procrustes(X, Y)

    Inputs:
    ------------
    X, Y
        matrices of target and input coordinates. they must have equal
        numbers of  points (rows), but Y may have fewer dimensions
        (columns) than X.

    scaling
        if False, the scaling component of the transformation is forced
        to 1

    reflection
        if 'best' (default), the transformation solution may or may not
        include a reflection component, depending on which fits the data
        best. setting reflection to True or False forces a solution with
        reflection or no reflection respectively.

    Outputs
    ------------
    d
        the residual sum of squared errors, normalized according to a
        measure of the scale of X, ((X - X.mean(0))**2).sum()

    Z
        the matrix of transformed Y-values

    tform
        a dict specifying the rotation, translation and scaling that
        maps X --> Y

    """

    n, m = X.shape
    ny, my = Y.shape

    muX = X.mean(0)
    muY = Y.mean(0)

    X0 = X - muX
    Y0 = Y - muY

    ssX = (X0 ** 2.).sum()
    ssY = (Y0 ** 2.).sum()

    # centred Frobenius norm
    normX = np.sqrt(ssX)
    normY = np.sqrt(ssY)

    # scale to equal (unit) norm
    X0 /= normX
    Y0 /= normY

    if my < m:
        Y0 = np.concatenate((Y0, np.zeros(n, m - my)), 0)

    # optimum rotation matrix of Y
    A = np.dot(X0.T, Y0)
    U, s, Vt = np.linalg.svd(A, full_matrices=False)
    V = Vt.T
    T = np.dot(V, U.T)

    if reflection != 'best':

        # does the current solution use a reflection?
        have_reflection = np.linalg.det(T) < 0

        # if that's not what was specified, force another reflection
        if reflection != have_reflection:
            V[:, -1] *= -1
            s[-1] *= -1
            T = np.dot(V, U.T)

    traceTA = s.sum()

    if scaling:

        # optimum scaling of Y
        b = traceTA * normX / normY

        # standarised distance between X and b*Y*T + c
        d = 1 - traceTA ** 2

        # transformed coords
        Z = normX * traceTA * np.dot(Y0, T) + muX

    else:
        b = 1
        d = 1 + ssY / ssX - 2 * traceTA * normY / normX
        Z = normY * np.dot(Y0, T) + muX

    # transformation matrix
    if my < m:
        T = T[:my, :]
    c = muX - b * np.dot(muY, T)

    # transformation values
    tform_mat = np.zeros((4, 4))
    tform_mat[3, 3] = 1
    tform_mat[:3, :3] = T.T*b
    tform_mat[:3, 3] = c
    tform = {'rotation': T, 'scale': b, 'translation': c}

    return d, Z, tform, tform_mat


def align_camera_traj(src_camera_traj, tar_camera_traj):
    """
    align the src point cloud to target point cloud
    """
    # convert to numpy array
    src_idx_key_map = {}
    src_camera_pc = np.zeros(
        [len(src_camera_traj.keys()), 3], dtype=np.float64)
    counter = 0
    for key, value in src_camera_traj.items():
        src_idx_key_map[counter] = key
        src_camera_pc[counter][:] = [value[0], value[1], value[2]]
        counter = counter + 1

    tar_camera_pc = np.zeros(
        [len(tar_camera_traj.keys()), 3], dtype=np.float64)
    counter = 0
    for key, value in tar_camera_traj.items():
        tar_camera_pc[counter][:] = [value[0], value[1], value[2]]
        counter = counter + 1

    # move to orgian of coordinate system
    src_camera_pc_centroid = np.mean(src_camera_pc, axis=0)
    src_camera_pc = src_camera_pc - src_camera_pc_centroid
    tar_camera_pc_centroid = np.mean(tar_camera_pc, axis=0)
    tar_camera_pc_new = tar_camera_pc - tar_camera_pc_centroid

    # the scale
    src_camera_pc_norm = np.mean(np.linalg.norm(src_camera_pc))
    tar_camera_pc_norm = np.mean(np.linalg.norm(tar_camera_pc_new))
    src_camera_pc = src_camera_pc * (tar_camera_pc_norm / src_camera_pc_norm)

    # ICP get transformation
    H = None
    if np.shape(src_camera_pc)[0] > np.shape(tar_camera_pc_new)[0]:
        H = np.dot(src_camera_pc[:np.shape(tar_camera_pc_new)[0],:].T, tar_camera_pc_new)
    elif np.shape(src_camera_pc)[0] < np.shape(tar_camera_pc_new)[0]:
        H = np.dot(src_camera_pc.T, tar_camera_pc_new[:np.shape(src_camera_pc)[0], :])
    else:
        H = np.dot(src_camera_pc.T, tar_camera_pc_new)
    U, S, Vt = np.linalg.svd(H)
    rotation_matrix = np.dot(Vt.T, U.T)    # rotation matrix

    # translation
    src_camera_pc_scaled_centroid = np.mean(src_camera_pc, axis=0)
    tar_camera_pc_scaled_centroid = np.mean(tar_camera_pc_new, axis=0)
    t = tar_camera_pc_scaled_centroid.T - np.dot(rotation_matrix, src_camera_pc_scaled_centroid.T)

    # update the camera pose
    src_camera_pc_new = (np.dot(rotation_matrix, src_camera_pc.T) +
                         t[:, np.newaxis] + tar_camera_pc_centroid[:, np.newaxis]).T
    # plot_camera_position(src_camera_pc_new, tar_camera_pc)

    src_camera_traj_new = {}
    # convert to original format
    for idx, key in src_idx_key_map.items():
        camera_pose = []
        # new position
        camera_pose = src_camera_pc_new[idx].tolist()
        # update rotation
        rotation_old_euler = src_camera_traj[key][3:]
        rotation_old = R.from_euler('xyz', rotation_old_euler, degrees=True).as_matrix()
        rotation_new_mat = np.dot(rotation_matrix, rotation_old)
        camera_pose = camera_pose + R.from_matrix(rotation_new_mat).as_euler("xyz", degrees=True).tolist()

        src_camera_traj_new[key] = camera_pose

    return src_camera_traj_new


def procrustes(input, target):
    src_camera_pc_centroid = np.mean(input, axis=0)
    tar_camera_pc_centroid = np.mean(target, axis=0)
    translation = tar_camera_pc_centroid - src_camera_pc_centroid

    src_dist_centroid = np.mean(np.linalg.norm(input - src_camera_pc_centroid, axis=1))
    tar_dist_centroid = np.mean(np.linalg.norm(target - tar_camera_pc_centroid, axis=1))
    scale = tar_dist_centroid/src_dist_centroid

    rot, _ = scipy.linalg.orthogonal_procrustes(input, target)

    transformed_input = np.dot(rot, input.T).T * scale + translation
    return transformed_input


scan_path = "/media/manuel/Elements/PHD/matterport/data/v1/scans"
sequences = [os.path.basename(os.path.dirname(element)) for element in glob.glob(os.path.join(scan_path, "*", ""))]
sequences.sort()
for idx_building, seq in enumerate(sequences):
    print(seq)
    openMVG_poses_file = os.path.join(scan_path, "{}/camera_params_estimation.txt".format(seq))
    GT_poses_file = os.path.join(scan_path, "{}/blender_GT_cam_poses.txt".format(seq))

    # Dont use skybox cam parameters
    idx_skybox = get_skybox_idx(GT_poses_file)
    GT_poses = load_camposes(GT_poses_file, idx_skybox)
    openMVG_poses = load_camposes(openMVG_poses_file, idx_skybox)

    openMVG_poses_full = load_camposes(openMVG_poses_file, -1)

    GT_poses_filtered = {x: GT_poses[x] for x in GT_poses if x in openMVG_poses_full.keys()}

    if not (GT_poses_filtered.keys().__eq__(openMVG_poses.keys())):
        continue

    data_openMVG = per_room_poses(openMVG_poses)
    data_GT = per_room_poses(GT_poses_filtered)

    output_procrustes = os.path.join(os.path.dirname(openMVG_poses_file), "SfM_poses_procrustes.txt")

    params = []
    for key in data_GT.keys():
        if data_openMVG[key].shape[0] < 2:
            continue
        d, Z_matlab, tform, tform_mat = procrustes_matlab(data_GT[key][:, :3], data_openMVG[key][:, :3], reflection=False)
        hom_openMVG = np.hstack((data_openMVG[key][:, :3], np.ones((data_openMVG[key].shape[0], 1))))
        #check = tform['scale'] * data_openMVG[key][:, :3] @ tform['rotation'] + tform['translation']
        #check2 = (tform_mat @ hom_openMVG.T).T

        tform_poses = []
        for idx in range(0, int(idx_skybox) + 1):
            key_with_idx = "{}_{:02}_rgb".format(key, idx)
            if key_with_idx not in openMVG_poses_full.keys():
                continue
            # c2w = [R^t C; 0 1]
            c2w = np.zeros((4, 4))
            c2w[:3, :3] = R.from_quat(openMVG_poses_full[key_with_idx][3:]).as_matrix().T   # transposed cause openMVG pose is [R C] instead of [R^t C]
            c2w[:3, 3] = openMVG_poses_full[key_with_idx][:3]
            c2w[3, 3] = 1
            tform_pose = tform_mat @ c2w @ np.diagflat(np.array([1, -1, -1, 1]))
            rot_new = R.from_matrix(tform_pose[:3, :3]).as_quat()
            center_new = tform_pose[:3, 3]
            params.append("0 {}_{:02}_rgb {} {} {} {} {} {} {}\n".format(key, idx, center_new[0], center_new[1],
                                                                         center_new[2], rot_new[0], rot_new[1],
                                                                         rot_new[2], rot_new[3]))

    with open(output_procrustes, 'w') as params_file:
        params_file.write("#index filename center.x center.y center.z quat_wc.x quat_wc.y quat_wc.z quat_wc.w\n")
        for pose in params:
            params_file.write(pose)


