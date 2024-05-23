from pyrcareworld.envs import RCareWorld, DressingEnv

# from pyrcareworld.envs import DressingEnv
import numpy as np
import pybullet as p
import math
import open3d as o3d
from pyrcareworld.utils.depth_processor import *
import time

env = DressingEnv()

target = env.create_object(id=101010, name="Cube", is_in_scene=True)

# creates visualization
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# initialize the point cloud in the window and save the capture to a pcd file
# r = env.camera.getRGB("wh")
# d = env.camera.getDepthEXR("wh")
# pcd = image_array_to_point_cloud(r, d, 60, env.local_to_world_matrix)
# pcd = pcd.voxel_down_sample(voxel_size=0.02)
# vis.add_geometry(pcd)
# pcd = env.camera.arraysToPointCloud("data/point_cloud.pcd")


while True:
    env.move_wheelchair(30)

    # the below code can be used to continually refresh the window with point cloud
    # pcd2 = image_array_to_point_cloud(r2, d2, 60, env.local_to_world_matrix)
    # # pcd2 = pcd2.voxel_down_sample(voxel_size=0.02)
    # pcd.points = pcd2.points
    # pcd.colors = pcd2.colors
    # vis.update_geometry(pcd)
    # vis.poll_events()
    # vis.update_renderer()
    env.step()
