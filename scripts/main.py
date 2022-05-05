import numpy as np
import cv2
import argparse
# from utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix
# import trimesh
# from policies import GraspingPolicy
# import matplotlib.pyplot as plt


try:
    import rospy
    import tf
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Image, CameraInfo
    from baxter_interface import gripper as baxter_gripper
    from intera_interface import gripper as sawyer_gripper
    from path_planner import PathPlanner
    ros_enabled = True
except:
    print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
    ros_enabled = False

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-obj', type=str, default='pawn', help=
        """Which Object you\'re trying to pick up.  Options: nozzle, pawn, cube.  
        Default: pawn"""
    )
    parser.add_argument('-n_vert', type=int, default=1000, help=
        'How many vertices you want to sample on the object surface.  Default: 1000'
    )
    parser.add_argument('-n_facets', type=int, default=32, help=
        """You will approximate the friction cone as a set of n_facets vectors along 
        the surface.  This way, to check if a vector is within the friction cone, all 
        you have to do is check if that vector can be represented by a POSITIVE 
        linear combination of the n_facets vectors.  Default: 32"""
    )
    parser.add_argument('-n_grasps', type=int, default=500, help=
        'How many grasps you want to sample.  Default: 500')
    parser.add_argument('-n_execute', type=int, default=5, help=
        'How many grasps you want to execute.  Default: 5')
    parser.add_argument('-metric', '-m', type=str, default='compute_force_closure', help=
        """Which grasp metric in grasp_metrics.py to use.  
        Options: compute_force_closure, compute_gravity_resistance, compute_robust_force_closure"""
    )
    parser.add_argument('-arm', '-a', type=str, default='right', help=
        'Options: left, right.  Default: right'
    )
    parser.add_argument('-robot', type=str, default='baxter', help=
        """Which robot you're using.  Options: baxter, sawyer.  
        Default: baxter"""
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

    if args.obj != 'cube':
        # Mesh loading and pre-processing
        # mesh = trimesh.load_mesh("objects/{}.obj".format(args.obj))
        mesh = trimesh.load_mesh("../objects/{}.obj".format(args.obj))
        # Transform object mesh to world frame
        T_world_obj = lookup_transform(args.obj) 
        # T_world_obj = np.eye(4)
        mesh.apply_transform(T_world_obj)
        mesh.fix_normals()
    else:
        camera_frame = ''
        if args.robot == 'baxter':
            camera_topic = '/cameras/left_hand_camera/image'
            camera_info = '/cameras/left_hand_camera/camera_info'
            camera_frame = '/left_hand_camera'
            locate_cube(camera_topic, camera_info, camera_frame)
        elif args.robot == 'sawyer':
            camera_topic = '/usb_cam/image_raw'
            camera_info = '/usb_cam/camera_info'
            camera_frame = '/usb_cam'
        else:
            print("Unknown robot type!")
            rospy.execute_grasp()
    grasping_policy = GraspingPolicy(
        args.n_vert, 
        args.n_grasps, 
        args.n_execute, 
        args.n_facets, 
        args.metric
    )

    # Each grasp is represented by T_grasp_world, a RigidTransform defining the 
    # position of the end effector
    grasp_vertices_total, grasp_poses = grasping_policy.top_n_actions(mesh, args.obj)

    if not args.sim:
        # Execute each grasp on the baxter / sawyer
        if args.robot == "baxter":
            gripper = baxter_gripper.Gripper(args.arm)
            planner = PathPlanner('{}_arm'.format(args.arm))
        elif args.robot == "sawyer":
            gripper = sawyer_gripper.Gripper("right")
            planner = PathPlanner('{}_arm'.format("right"))
        else:
            print("Unknown robot type!")
            rospy.shutdown()

    for grasp_vertices, grasp_pose in zip(grasp_vertices_total, grasp_poses):
        grasping_policy.visualize_grasp(mesh, grasp_vertices, grasp_pose)
        if not args.sim:
            repeat = True
            while repeat:
                execute_grasp(grasp_pose, planner, gripper)
                repeat = raw_input("repeat? [y|n] ") == 'y'
