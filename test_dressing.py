from pyrcareworld.envs import RCareWorld, DressingEnv

# from pyrcareworld.envs import DressingEnv
import numpy as np
import pybullet as p
import math

env = DressingEnv()

print(env.get_bed_angle())
env.set_bed_angle(20)
print(env.get_bed_angle())
env.set_bed_angle(60)
print(env.get_bed_angle())
env.set_bed_angle(10)
print(env.get_bed_angle())
print(env.camera.getPosition())
print(env.camera.getCameraInfo())
# np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/rgb.npy",
#     env.camera.getRGB("wh"),
# )
# np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/depth.npy",
#     env.camera.getDepthEXR("wh"),
# )
# np.save(
#     "/home/hcn9/rcareworld_workspace/rcare_py/pyrcareworld/Dressing/world.npy",
#     env.camera.getLocalToWorldMatrix(),
# )
# pcd = env.camera.arraysToPointCloud("data/point_cloud.pcd")
