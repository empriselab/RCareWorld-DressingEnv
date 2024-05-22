from pyrcareworld.envs import RCareWorld, DressingEnv

# from pyrcareworld.envs import DressingEnv
import numpy as np
import pybullet as p
import open3d as o3d
from pyrcareworld.utils.depth_processor import *
import time

env = RCareWorld()

intrinsic_matrix = [600, 0, 0, 0, 600, 0, 240, 240, 1]
camera = env.create_camera(
    id=123456,
    name="scene_camera",
    intrinsic_matrix=intrinsic_matrix,
    width=480,
    height=480,
)
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# initialize the point cloud in the window
pcd = camera.arraysToPointCloud("data/hole_point_cloud.pcd")

# r = camera.getRGB("wh")
# d = camera.getDepthEXR("wh")
# local_to_world_matrix = camera.getLocalToWorldMatrix()
# pcd = image_array_to_point_cloud(r, d, 60, local_to_world_matrix)
# pcd = pcd.voxel_down_sample(voxel_size=0.02)
# vis.add_geometry(pcd)
# vis.update_geometry(pcd)
# o3d.visualization.draw_geometries([pcd])

# fetch the updated image

start_time = time.time()
x = 1  # displays the frame rate every 1 second
counter = 0

while True:
    # position = target.getPosition()
    # env.robot.directlyMoveTo(position)
    # r2 = camera.getRGB("wh")
    # d2 = camera.getDepthEXR("wh")
    # pcd2 = image_array_to_point_cloud(r2, d2, 60, local_to_world_matrix)
    # pcd2 = pcd2.voxel_down_sample(voxel_size=0.02)
    # pcd.points = pcd2.points
    # pcd.colors = pcd2.colors
    # vis.update_geometry(pcd)
    # vis.poll_events()
    # vis.update_renderer()
    counter += 1
    if (time.time() - start_time) > x:
        print("FPS: ", counter / (time.time() - start_time))
        counter = 0
        start_time = time.time()
    env._step()
