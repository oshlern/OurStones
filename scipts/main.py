#!/usr/bin/env python3

import rospy
import tf

from path_planner import PathPlanner
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move import TrajectoryClient

import numpy as np

from utils import lookup_transform, quaternion_from_matrix

from ur5 import UR5Robot
from sensor_msgs.msg import JointState



rospy.init_node('our_node')
listener = tf.TransformListener()
planner = PathPlanner('manipulator')
# client = TrajectoryClient()

def move_delta(dt, max_veloicty=0.1):
    H = lookup_transform(listener, "wrist_3_link", "base")
    t, R = H[:3, -1], H[:3, :3]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = t + dt
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_matrix(R)
    pose_list = [pose]
    duration_list = [np.max(dt) / max_veloicty]
    client.send_cartesian_trajectory(pose_list, duration_list)

def main():

    # sr = UR5Robot()
    # rospy.Subscriber("/joint_states", JointState, sr.update_joints)
    # for i in range(20):
    #     print(sr.jacobian)
    #     input()

    ##### Cartesian controller
    # H = lookup_transform(listener, "wrist_3_link", "base")
    # t, R = H[:3, -1], H[:3, :3]
    #
    # pose_list = [
    #     Pose(Vector3(t[0], t[1], t[2] + 0.05), Quaternion(0, 0, 0, 1)),
    # ]
    # duration_list = [5]

    # move_delta(np.array([0.05,0,0]))
    # input()

    # client.send_cartesian_trajectory(pose_list, duration_list)


    pose = Pose()
    neutral = np.array([[ 0.99858429,  0.01964435,  0.04943188, -0.12249568],
                        [ 0.05016468, -0.65684315, -0.75235669, -0.47999735],
                        [ 0.01768943,  0.75377131, -0.6568987,   0.4463452 ],
                        [ 0.,          0.,          0.,          1.        ]])

    t = neutral[:3, -1]
    R = neutral[:3, :3]

    R = np.array([[0,  1, 0],
                  [-1, 0, 0],
                  [0,  0, -1]])
    t = np.array([0, -0.4, 0.4]) # Above grabbing pose
    t = np.array([0, -0.4, 0.2239]) # At grabbing pose



    # Can subscribe to /scaled_pos_joint_traj_controller/state or /joint_states for current state (control_msgs/JointTrajectoryControllerState)
    jointCmdPub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, latch=True, queue_size=1)
    jagnles = []
    msg = JointTrajectory()
    msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    pts = JointTrajectoryPoint()
    pts.positions = [-1.6, -0.6, -1.85, 0.35, -1.57, 2.87]
    pts.time_from_start.secs = 1
    msg.points = [pts]
    msg.header.stamp = rospy.Time.now()
    # jointCmdPub.publish(msg)
    # rospy.sleep(0.3)
    # print("Message sent")

    #
    # t = np.array([0, -0.5, 0.5])
    # R = np.array([[0.99797485, -0.02068542, -0.06015238],
    #               [0.02174908, -0.77769161,  0.62826964],
    #               [-0.05977603, -0.62830556, -0.77566678]])

    pose.position.x, pose.position.y, pose.position.z = t
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_matrix(R)

    # Add ground plane
    poseStamped = PoseStamped()
    poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z = (0, 0, -0.06)
    poseStamped.header.frame_id = 'base_link'
    planner.add_box_obstacle((2, 2, 0.1), "box0", poseStamped)

    # Add side plane
    poseStamped = PoseStamped()
    poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z = (0, 0.5, 1)
    poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w = (0.7068252, 0, 0, 0.7073883)
    poseStamped.header.frame_id = 'base_link'
    planner.add_box_obstacle((2, 2, 0.1), "box1", poseStamped)

    _, plan, _, _ = planner.plan_to_pose(pose)
    if input("Execute Plan? (y/n)") == 'y':
        planner.execute_plan(plan)
        rospy.sleep(1.0)

    # print("Current robot location:", lookup_transform(listener, "wrist_3_link", "base_link"))


if __name__=="__main__":
    main()
