
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import rtde_control
import rtde_receive

# Examples + API Docs
# https://gitlab.com/sdurobotics/ur_rtde/-/tree/master/examples/py
# https://sdurobotics.gitlab.io/ur_rtde/api/api.html?highlight=movej#_CPPv4N7ur_rtde4RTDE12RobotCommand4Type5MOVEJE

DOWN_ROTATION = R.from_rotvec([2.925, 1.139, 0.028])
DEFAULT_POSITION = [0.054, 0.379, 0.457]
DEFAULT_JANGLES = [1.722, -1.289, -1.746, -1.676, 1.553, 5.692]

class Arm(object):
    def __init__(self, ip="172.22.22.2"):
        self.ip = ip
        self.rtde_c = rtde_control.RTDEControlInterface(self.ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip)

        self.MIN_X, self.MAX_X = -0.3, 0.3
        self.MIN_Y, self.MAX_Y = 0., 0.6
        self.MIN_Z, self.MAX_Z = 0.272, 0.8

        self.speed = 0.02
        self.fast_speed = 0.2
        self.accel = 0.05
        print("Initialized Arm")

    def read_pose(self):
        pose = self.rtde_r.getActualTCPPose()
        t = pose[:3]
        r = R.from_rotvec(pose[3:])
        return t, r

    def read_velocity(self):
        vel = self.rtde_r.getActualTCPSpeed()
        v = vel[:3]
        w = vel[3:]
        return v, w

    def read_joint_angles(self):
        joint_angles = self.rtde_r.getActualQ()
        return joint_angles

    def read_joint_velocities(self):
        joint_velocities = self.rtde_r.getActualQd()
        return joint_velocities

    def to_joint_position(self, pose, speed=1.05, acceleration=1.4):
        return self.rtde_c.moveJ(pose, speed, acceleration)

    def follow_position_trajectory(self, trajectory):
        # This will require some work to make nice
        # See https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/examples/py/move_path_async_example.py
        raise NotImplementedError
        # return self.rtde_c.movePath(trajectory)

    def move_to(self, position, rotation=DOWN_ROTATION, position_name="", silent=False, fast=False):

        rotation = rotation.as_rotvec()
        if self.out_of_bounds(position, rotation):
            raise Exception("Out of bounds with position: {}, rotation: {}".format(position, rotation))
        if not silent:
            print("Moving to {} position: {}".format(position_name, position))

        speed = self.fast_speed if fast else self.speed
        self.rtde_c.moveL(list(position) + list(rotation), speed, self.accel)

    def move_until_contact(self, velocity):
        self.rtde_c.moveUntilContact(velocity)

    def set_vel(self, vel, dt=0.1):
        # See https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/examples/py/speedj_example.py
        self.rtde_c.speedL(vel, self.accel, dt)

    def set_joint_vel(self, vel, dt=0.1):
        self.rtde_c.speedL(vel, self.accel, dt)

    def stop_speed_control(self):
        self.rtde_c.speedStop()

    def tuck(self, tcp=False, fast=False):
        if tcp:
            self.move_to(DEFAULT_POSITION, DOWN_ROTATION, position_name="tucked", fast=fast)
        else:
            self.to_joint_position(DEFAULT_JANGLES)

    def move_relative(self, displacement):
        cur_position, cur_rotation = self.read_pose()
        target_position = cur_position + displacement
        self.move_to(target_position, cur_rotation, silent=True)

    def out_of_bounds(self, position, rotation):
        out = False
        if not (self.MIN_X < position[0] < self.MAX_X):
            out = True
        if not (self.MIN_Y < position[1] < self.MAX_Y):
            out = True
        if not (self.MIN_Z < position[2] < self.MAX_Z):
            out = True
        return out
        # TODO: account for orientation
