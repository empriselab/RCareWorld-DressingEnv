from pyrcareworld.envs import RCareWorld
import numpy as np
import pybullet as p
import math
import open3d as o3d
from pyrcareworld.utils.depth_processor import *


class DressingEnv(RCareWorld):

    # object for cloth
    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        custom_channels: list = [],
        assets: list = [],
        **kwargs
    ):
        RCareWorld.__init__(
            self,
            executable_file=executable_file,
            scene_file=scene_file,
            custom_channels=custom_channels,
            assets=assets,
            **kwargs,
        )

        # create scene objects here as class variables
        self.robot_id = 639787
        self.bed_id = 234567
        self.camera_id = 567890
        self.robot_dof = 7
        self.bed = self.create_bed(
            id=self.bed_id,
            name="bed_actuation",
            is_in_scene=True,
        )
        self.robot = self.create_robot(
            id=self.robot_id,
            gripper_list=["6397870"],
            robot_name="franka-panda",
            base_pos=[0, 0, 1],
        )

        self.intrinsic_matrix = [600, 0, 0, 0, 600, 0, 240, 240, 1]
        self.camera = self.create_camera(
            id=self.camera_id,
            name="scene_camera",
            intrinsic_matrix=self.intrinsic_matrix,
            width=480,
            height=480,
        )
        self.local_to_world_matrix = self.camera.getLocalToWorldMatrix()
        self.wheelchair = self.create_robot(
            id=98765,
            robot_name="wheelchair",
        )

    def get_state(self):
        return self.robot.getRobotState()

    def get_robot_joint_positions(self):
        return self.instance_channel.data[self.robot_id]["joint_positions"]

    def get_robot_joint_velocities(self):
        return self.instance_channel.data[self.robot_id]["joint_velocities"]

    def get_robot_joint_accelerations(self):
        return self.instance_channel.data[self.robot_id]["joint_accelerations"]

    def set_robot_joint_position(
        self, joint_positions=None, joint_velocities=None, joint_accelerations=None
    ):

        if joint_positions is not None:
            self.instance_channel.set_action(
                "SetJointPositionDirectly",
                id=self.robot_id,
                joint_positions=list(joint_positions[0 : self.robot_dof]),
            )

        if joint_velocities is not None:
            self.instance_channel.set_action(
                "SetJointVelocity",
                id=self.robot_id,
                joint_velocitys=list(joint_velocities[0 : self.robot_dof]),
            )

        if joint_accelerations is not None:
            self.instance_channel.set_action(
                "SetJointAcceleration",
                id=self.robot_id,
                joint_accelerations=list(joint_accelerations[0 : self.robot_dof]),
            )

    def set_bed_angle(self, angle, duration=25):
        """
        @param velocity: the angle at which to move the hospital bed to
        """
        self.bed.setActuationAngle(angle, duration)

    def get_bed_angle(self):
        return self.bed.getCurrentAngle()

    def step(self):
        self._step()

    def move_wheelchair(self, velocity):
        """
        @param velocity: the velocity at which to move the wheelchair
        """
        self.wheelchair.setJointVelocitiesDirectly([velocity, 0])

    def stop_wheelchair(self):
        """
        stops the wheelchair's movement
        """
        self.wheelchair.setJoinVelocitiesDirectly([0, 0])
