import numpy as np
import argparse
import rospy
from arm import Arm
from camera import Camera
from ft_sensor import ForceTorqueSensor
from gripper import Gripper
from robot import Robot

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', type=str, default='172.22.22.2', help=
        """UR5 ip.
        Default: 172.22.22.2"""
    )
    parser.add_argument('--sim', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is
        so you can run this outside of the lab"""
    )
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    if args.debug:
        np.random.seed(0)

    if not args.sim:
        # Init rospy node (so we can use ROS commands)
        rospy.init_node('our_node')

    gripper = Gripper()
    arm = Arm(ip=args.ip)
    camera = Camera()
    ft_sensor = ForceTorqueSensor()

    robot = Robot(gripper, arm, camera, ft_sensor)
    robot.main()
