import numpy as np
from matplotlib import pyplot as PLT
from pyrcareworld.utils.depth_processor import *
import matplotlib.pyplot as plt
import open3d as o3d

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("data/hole_point_cloud.pcd")
    o3d.visualization.draw_geometries([pcd])
