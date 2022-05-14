
import numpy as np
from geometry_msgs.msg import Pose
import rtde_control
import rtde_receive

DOWN_ROTATION = [2.925, 1.139, 0.028]
DEFAULT_POSITION = [0.054, 0.379, 0.457]

class Arm(Object):
    def __init__(self, ip="172.22.22.2"):
        self.ip = ip
        self.rtde_c = rtde_control.RTDEControlInterface(self.ip)
        self.rtde_r = rtde_receive.RTDECReveiveInterface(self.ip)

        self.MIN_X, self.MAX_X = -0.3, 0.3
        self.MIN_Y, self.MAX_Y = 0., 0.6
        self.MIN_Z, self.MAX_Z = 0.272, 0.8

        self.speed = 0.01
        self.accel = 0.25

    def read_pose(self):
        pose = self.rtde_r.getActualTCPPose()
        position = pose[:3]
        rotation = pose[3:]
        return position, rotation

    def move_to(self, position, rotation=DOWN_ROTATION, position_name=""):
        if self.out_of_bounds(position, rotation):
            raise Exception("Out of bounds with position: {}, rotation: {}".format(position, rotation))
        # pose = Pose()
        # pose.position.x, pose.position.y, pose.position.z = position
        # pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        # plan = self.planner.plan_to_pose(pose)
        # self.planner.execute_plan(plan)
        print("Moving to {} position: {}".format(position_name, position))

        self.rtde_c.moveL(list(position) + list(rotation), self.speed, self.accel)

    def tuck(self):
        self.move_to(DEFAULT_POSITION, DOWN_ROTATION, position_name="tucked")

    def move_relative(self, displacement):
        dx, dy, dz = displacement
        cur_position, cur_rotation = self.read_pose()
        target_position = cur_position
        target_position[0] += dx
        target_position[1] += dy
        target_position[2] += dz
        self.move_to(target_position, cur_rotation)

    # def set_vel(self, vx, vy, vz): (rotation too)

    def out_of_bounds(self, position, rotation):
        out = False
        if self.MIN_X < position[0] < self.MAX_X:
            out = True
        if self.MIN_Y < position[1] < self.MAX_Y:
            out = True
        if self.MIN_Z < position[2] < self.MAX_Z:
            out = True
        return out
        # TODO: account for orientation
