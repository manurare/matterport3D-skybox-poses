import os
import numpy as np
import matplotlib.pyplot as plt
from struct import unpack
import glob
import matplotlib
matplotlib.use("Qt5Agg")


def read_dpt(dpt_file_path):
    """read depth map from *.dpt file.

    :param dpt_file_path: the dpt file path
    :type dpt_file_path: str
    :return: depth map data
    :rtype: numpy
    """
    TAG_FLOAT = 202021.25  # check for this when READING the file

    ext = os.path.splitext(dpt_file_path)[1]

    assert len(ext) > 0, ('readFlowFile: extension required in fname %s' % dpt_file_path)
    assert ext == '.dpt', exit('readFlowFile: fname %s should have extension ''.flo''' % dpt_file_path)

    fid = None
    try:
        fid = open(dpt_file_path, 'rb')
    except IOError:
        print('readFlowFile: could not open %s', dpt_file_path)

    tag = unpack('f', fid.read(4))[0]
    width = unpack('i', fid.read(4))[0]
    height = unpack('i', fid.read(4))[0]

    assert tag == TAG_FLOAT, ('readFlowFile(%s): wrong tag (possibly due to big-endian machine?)' % dpt_file_path)
    assert 0 < width and width < 100000, ('readFlowFile(%s): illegal width %d' % (dpt_file_path, width))
    assert 0 < height and height < 100000, ('readFlowFile(%s): illegal height %d' % (dpt_file_path, height))

    # arrange into matrix form
    depth_data = np.fromfile(fid, np.float32)
    depth_data = depth_data.reshape(height, width)

    fid.close()

    return depth_data


if __name__ == "__main__":
    # Data dir
    data_dir = "/home/manuel/Desktop/PHD/code/test_openMVG/Image_datasets/Preprocessed/Wulongting/GT"
    gt_depthmaps = glob.glob(os.path.join(data_dir, "*.dpt"))
    for depthmap in gt_depthmaps:
        gt = read_dpt(depthmap)
        depth_values = np.where(gt != 0.)
        plt.imshow(gt, cmap="turbo")
        plt.plot(depth_values[1], depth_values[0], 'r*')
        plt.show()