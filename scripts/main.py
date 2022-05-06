import numpy as np
# import cv2
import argparse
import rospy
# from src.objects import Robot
# from utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix
# import trimesh
# from policies import GraspingPolicy
# import matplotlib.pyplot as plt

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
        rospy.init_node('dummy_tf_node')

    gripper = Gripper()
    arm = Arm(ip=args.ip)
    camera = Camera()
    ft_sensor = ForceTorqueSensor()

    robot = Robot(gripper, arm, camera, ft_sensor)

    arm.tuck()



    # robot.main()

        # camera_frame = ''
        # if args.robot == 'baxter':
        #     camera_topic = '/cameras/left_hand_camera/image'
        #     camera_info = '/cameras/left_hand_camera/camera_info'
        #     camera_frame = '/left_hand_camera'
        #     locate_cube(camera_topic, camera_info, camera_frame)
        #     rospy.execute_grasp()

    # for grasp_vertices, grasp_pose in zip(grasp_vertices_total, grasp_poses):
        # grasping_policy.visualize_grasp(mesh, grasp_vertices, grasp_pose)
        # if not args.sim:
        #     repeat = True
        #     while repeat:
        #         execute_grasp(grasp_pose, planner, gripper)
        #         repeat = raw_input("repeat? [y|n] ") == 'y'
