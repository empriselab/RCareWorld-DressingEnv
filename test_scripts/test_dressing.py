from pyrcareworld.envs import RCareWorld, DressingEnv

# from pyrcareworld.envs import DressingEnv
import numpy as np
import pybullet as p
import math
import open3d as o3d
from pyrcareworld.utils.depth_processor import *
import time

# this test script is to be used with the scene called Test Robot with Bed in the dressing repo under the python_api branch of the Dressing Scenes repository.
env = DressingEnv()

target = env.create_object(id=101010, name="Cube", is_in_scene=True)
vis = o3d.visualization.Visualizer()
vis.create_window()

# initialize the point cloud in the window
r = env.camera.getRGB("wh")
d = env.camera.getDepthEXR("wh")
pcd = image_array_to_point_cloud(r, d, 60, env.local_to_world_matrix)
# pcd = pcd.voxel_down_sample(voxel_size=0.02)
vis.add_geometry(pcd)

# # reset the camera view in the visualization
# ctr = vis.get_view_control()
# parameters = o3d.io.read_pinhole_camera_parameters(
#     "ScreenCamera_2024-04-25-00-59-37.json"
# )
# ctr.convert_from_pinhole_camera_parameters(parameters)
# print(env.get_bed_angle())
# env.set_bed_angle(20)
# print(env.get_bed_angle())
# env.set_bed_angle(60)
# print(env.get_bed_angle())
# env.set_bed_angle(10)
# print(env.get_bed_angle())
# print(env.camera.getPosition())
# print(env.camera.getCameraInfo())
# np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/rgb.npy",
#     env.camera.getRGB("wh"),
# )
# # np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/depth.npy",
#     env.camera.getDepthEXR("wh"),
# )
# np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/world.npy",
#     env.camera.getLocalToWorldMatrix(),
# )
# pcd = env.camera.arraysToPointCloud("data/point_cloud.pcd")

# fetch the updated image

start_time = time.time()
x = 1  # displays the frame rate every 1 second
counter = 0

while True:
    # position = target.getPosition()
    # env.robot.directlyMoveTo(position)
    r2 = env.camera.getRGB("wh")
    d2 = env.camera.getDepthEXR("wh")
    # pcd2 = image_array_to_point_cloud(r2, d2, 60, env.local_to_world_matrix)
    # # pcd2 = pcd2.voxel_down_sample(voxel_size=0.02)
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
    env.step()
