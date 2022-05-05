
import numpy as np
from geometry_msgs.msg import Pose


QUAT_DOWN = [0.727, -0.686, 0.038, 0.002]
DEFAULT_POSITION = [0.752, -0.35, 0.02]

class Arm(Object):
    def __init__(self, planner):
        self.planner = planner
        self.tuck_position = DEFAULT_POSITION
    
    def read_pose(self):
        pose = None # TODO
        position = pose[:3]
        quaternion = pose[3:]
        return position, quaternion

    def move_to(self, position, quaternion=QUAT_DOWN, position_name=""):
        if self.out_of_bounds(position, quaternion):
            raise Exception("Out of bounds with position: {}, quat: {}".format(position, quaternion))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        plan = self.planner.plan_to_pose(pose)
        print("Moving to {} position: {}".format(position_name, pose.position))
        self.planner.execute_plan(plan)

    def tuck(self):
        self.move_to(DEFAULT_POSITION, position_name="tucked")
    
    def move_relative(self, displacement):
        dx, dy, dz = displacement
        current_position, current_quat = self.read_pose()
        target_position = current_position
        target_position[0] += dx
        target_position[1] += dy
        target_position[2] += dz
        self.move_to(target_position, current_quat)
    
    # def set_vel(self, vx, vy, vz): (rotation too)

    def out_of_bounds(self, position, quaternion):
        out = False
        if self.MIN_X < position[0] < self.MAX_X:
            out = True
        if self.MIN_Y < position[1] < self.MAX_Y:
            out = True
        if self.MIN_Z < position[2] < self.MAX_Z:
            out = True
        return out
        # TODO: account for orientation 