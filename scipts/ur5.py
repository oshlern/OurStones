import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
import numpy as np


class UR5Robot:
    def __init__(self):
        # load in ros parameters
        self.baselink = "base"
        self.endlink = "wrist_3_link"
        flag, self.tree = kdl_parser.treeFromParam("/robot_description")

        # build kinematic chain and fk and jacobian solvers
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)
        self.jac_ee = kdl.ChainJntToJacSolver(chain_ee)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()
        self.joints = kdl.JntArray(self.num_joints)
        self.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    def update_joints(self, joint_msg):
        if self.joint_names[0] in joint_msg.name:
            for i, n in enumerate(self.joint_names):
                index = joint_msg.name.index(n)
                self.joints[i] = joint_msg.position[index]

            # frame = kdl.Frame()
            # self.fk_ee.JntToCart(self.joints, frame)
            # print(frame.p, frame.M)
            # jacobian = kdl.Jacobian(self.num_joints)
            # self.jac_ee.JntToJac(self.joints, jacobian)
            # print(np.hstack([jacobian.getColumn(i) for i in range(self.num_joints)]))

    @property
    def jacobian(self):
        jacobian = kdl.Jacobian(self.num_joints)
        self.jac_ee.JntToJac(self.joints, jacobian)
        return np.hstack([jacobian.getColumn(i) for i in range(self.num_joints)])

def main():
    rospy.init_node("jacobian")
    sr = UR5Robot()
    rospy.Subscriber("/joint_states", JointState, sr.update_joints)
    rospy.spin()

if __name__ == "__main__":
    main()
