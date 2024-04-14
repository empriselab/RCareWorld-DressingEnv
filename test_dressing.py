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
# print(env.camera.getCameraInfo())
# print(env.camera.getRGB("intrinsic"))

# env.camera.initializeDepthEXRWithIntrinsic()
# env.camera.initializeRGBWithIntrinsic()
# print(env.camera.getPointCloudWithDepth())
# while True:
# env.step()
