import numpy as np
from matplotlib import pyplot as PLT
from pyrcareworld.utils.depth_processor import *
import matplotlib.pyplot as plt
import open3d as o3d


if __name__ == "__main__":
    # d = np.load("data/depth.npy")
    r = np.load("data/rgb.npy")
    # local_to_world_matrix = np.load("data/world.npy")
    # intrinsic_matrix = [600, 0, 0, 0, 600, 0, 240, 240, 1]
    # nd_main_intrinsic_matrix = np.reshape(intrinsic_matrix, [3, 3]).T

    # rgb = o3d.geometry.Image(r)
    # depth = o3d.geometry.Image(d)
    # PLT.imshow(r)
    # PLT.show()
    pcd = o3d.io.read_point_cloud("data/point_cloud.pcd")
    # pcd = image_array_to_point_cloud(r, d, 60, local_to_world_matrix)
    o3d.visualization.draw_geometries([pcd])

    # test with just depth
    # pc = depth_to_point_cloud(d, 60)
    # x = pc[:, 0]
    # y = pc[:, 1]
    # z = pc[:, 2]
    # fig = plt.figure(figsize=(12, 7))
    # ax = fig.add_subplot(projection="3d")
    # img = ax.scatter(x, y, z)
    # plt.show()
