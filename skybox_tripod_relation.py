import numpy as np
import os
import glob
from scipy.spatial.transform import Rotation as R
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use("Qt5Agg")


def get_outliers(values, max_deviations=2):
    mean = np.mean(values)
    standard_deviation = np.std(values)
    distance_from_mean = abs(values - mean)
    outliers = distance_from_mean > max_deviations * standard_deviation
    idxs = np.where(outliers)[0]
    return outliers, idxs

def upright_rotation(vertical_cam_axis, vertical_world_axis):
    v = np.cross(vertical_cam_axis, vertical_world_axis)
    s = np.linalg.norm(v)
    c = np.dot(vertical_cam_axis, vertical_world_axis)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    r = np.eye(3) + vx + (vx @ vx) * (1 - c) / (s ** 2)
    return r


def plot_axis(ax, rots, idx=0, wrt=np.eye(3)):
    zero = np.zeros(3)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    custom_lines = [plt.Line2D([0], [0], color='k', lw=2),
                    plt.Line2D([0], [0], color='r', lw=2)]
    ax.legend(custom_lines, ["skybox", "closest tripod"])
    x, y, z = zip(zero, wrt[:, 0])
    ax.plot(x, y, z, 'k', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'X', 'y', color='k')
    x, y, z = zip(zero, -wrt[:, 1])
    ax.plot(x, y, z, 'k', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'Y', 'y', color='k')
    x, y, z = zip(zero, wrt[:, 2])
    ax.plot(x, y, z, 'k', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'Z', 'y', color='k')
    x, y, z = zip(zero, rots[idx, :, 0])
    ax.plot(x, y, z, 'r', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'X', 'y', color='r')
    x, y, z = zip(zero, -rots[idx, :, 1])
    ax.plot(x, y, z, 'r', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'Y', 'y', color='r')
    x, y, z = zip(zero, rots[idx, :, 2])
    ax.plot(x, y, z, 'r', zdir='y', linewidth=2)
    ax.text(x[-1], z[-1], y[-1], 'Z', 'y', color='r')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)


def plot_rotated_cs(rots, wrt=np.eye(3)):
    fig = plt.figure()
    if rots.ndim >= 3:
        for i in range(0, 3):
            for j in range(0, 6):
                idx = i*6 + j
                ax = fig.add_subplot(3, 6, idx+1, projection='3d')
                plot_axis(ax, rots, idx=idx, wrt=wrt)
    else:
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        plot_axis(ax, rots[None, ...], wrt=wrt)
    # plt.show()


def parse_tripod(conf_file, uuids):
    tripod_poses = {}
    with open(conf_file) as f:
        content = f.readlines()
    for uuid in uuids:
        uuid_imgs = [line.splitlines() for line in content if uuid in line]
        poses = [np.array(line[0].split(' ')[3:]).astype(np.float64).reshape(4, 4) for line in uuid_imgs]
        rots = np.array([R.from_matrix(pose[:3, :3]).as_quat() for pose in poses])
        centers = np.array([pose[:3, 3] for pose in poses])
        tripod_poses[uuid] = np.hstack((centers, rots))

    return tripod_poses


def parse_skybox(conf_file):
    skybox_poses = {}
    with open(conf_file) as f:
        content = [line.rstrip() for line in f]
    uuids = [line.splitlines()[0].split(" ")[1].split("_")[0] for line in content if line[0][0] != "#"]
    uuids = np.unique(np.array(uuids))
    for uuid in uuids:
        for line in content:
            if uuid in line and "_18_rgb" in line:
                skybox_poses[uuid] = np.array(line.split(' ')[2:]).astype(np.float64)

    return uuids, skybox_poses


scan_path = "/media/manuel/Elements/PHD/matterport/data/v1/scans"
sequences = [os.path.basename(os.path.dirname(element)) for element in glob.glob(os.path.join(scan_path, "*", ""))]
sequences.sort()
for idx_building, seq in enumerate(sequences):
    print(seq)
    tripod_poses_p = os.path.join(scan_path, "{}/undistorted_camera_parameters/{}.conf".format(seq, seq))
    skybox_poses_p = os.path.join(scan_path, "{}/SfM_poses_procrustes.txt".format(seq))

    uuids, skybox_poses = parse_skybox(skybox_poses_p)
    tripod_poses = parse_tripod(tripod_poses_p, uuids)

    skybox_inferred_poses = {}
    center_dif = {}
    rot_difs = {}
    for key in skybox_poses.keys():
        sk = skybox_poses[key]
        tripod = tripod_poses[key]
        center_dif_per_sample = np.min(np.sum(np.abs(tripod[:, 0:3] - sk[0:3]), axis=1))
        argmin_sample = np.argmin(np.sum(np.abs(tripod[:, 0:3] - sk[0:3]), axis=1))
        center_dif_mean = np.sum(np.abs(np.mean(tripod[:, 0:3], axis=0) - sk[0:3]))
        center_dif_central_ring = np.sum(np.abs(np.mean(tripod[6:12, 0:3], axis=0) - sk[0:3]))
        center_dif[key] = np.hstack((center_dif_per_sample, argmin_sample, center_dif_mean, center_dif_central_ring))

        rot_sk = sk[3:]
        rot_tripod = tripod[:, 3:]
        upright_rots = [upright_rotation(R.from_quat(rot_tripod[i]).as_matrix() @ np.array([0, 1, 0]), np.array([0, 0, 1]))
                        for i in range(0, rot_tripod.shape[0])]
        upright_rots = np.stack(upright_rots)
        rot_tripod_wrt_sk = R.from_quat(rot_sk).as_matrix().T @ (upright_rots @ R.from_quat(rot_tripod).as_matrix())
        # plot_rotated_cs(rot_tripod_wrt_sk)
        # plot_rotated_cs(R.from_euler('XYZ', [0, np.pi*0.5, 0]).as_matrix() @ rot_tripod_wrt_sk[8])
        rot_dif = R.from_quat(rot_sk).as_matrix() @ (R.from_euler('XYZ', [0, 0, np.pi*0.5]).as_matrix() @ R.from_quat(rot_tripod[8]).as_matrix()).T
        ang_dif = np.degrees(np.arccos((np.trace(rot_dif) - 1)*0.5))
        rot_difs[key] = ang_dif
        inferred_rot = R.from_matrix(R.from_euler('XYZ', [0, 0, np.pi*0.5]).as_matrix() @ upright_rots[8] @ R.from_quat(rot_tripod[8]).as_matrix())
        inferred_pose = np.hstack((np.mean(tripod[6:12, 0:3], axis=0), inferred_rot.as_quat()))
        skybox_inferred_poses[key] = inferred_pose

    outliers, idxs = get_outliers(np.array(list(center_dif.values()))[:, 0])

    txt_file = os.path.join(scan_path, "{}/skybox_camera_parameters.txt".format(seq))
    with open(txt_file, 'w') as f:
        f.write("#index filename center.x center.y center.z quat_wc.x quat_wc.y quat_wc.z quat_wc.w\n")
        for idx, (key, value) in enumerate(skybox_poses.items()):
            if idx in idxs:
                continue
            f.write("{} {} {} {} {} {} {} {} {}\n".format(idx, key, value[0], value[1], value[2], value[3], value[4], value[5],
                                                          value[6]))

    # df = pd.DataFrame.from_dict(rot_difs, orient="index")
    # df.to_csv("/home/manuel/Desktop/data.csv")
    # plt.plot(range(0, center_dif.keys().__len__()), np.array(list(center_dif.values()))[:, 0], 'ro')
    # plt.plot(range(0, center_dif.keys().__len__()), np.array(list(center_dif.values()))[:, 2], 'bo')
    # plt.plot(range(0, center_dif.keys().__len__()), np.array(list(center_dif.values()))[:, 3], 'yo')
    # plt.legend(["per sample", "mean three rings", "mean central ring"])
    # plt.show()
    # print("hola")



