
import rospy
# import _ as robot_gripper # TODO

import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def open_gripper():
    action_name = 'command_robotiq_action'
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    robotiq_client.wait_for_server() # need to run roslaunch robotiq_2f_gripper_control robotiq_action_server.launch stroke:=0.140
    Robotiq.goto(robotiq_client, pos=0.140, force=100, block=True)


class Gripper(Object):
    def __init__(self):
        pass
        # self.gripper = robot_gripper.Gripper('right')


    def calibrate(self):
        print('Calibrating Gripper...')
        # self.gripper.calibrate()
        # rospy.sleep(1.0)
        self.close()
        self.open()
        print('Done!')
    
    def close(self):
        action_name = 'command_robotiq_action'
        robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        robotiq_client.wait_for_server() # need to run roslaunch robotiq_2f_gripper_control robotiq_action_server.launch stroke:=0.140
        Robotiq.goto(robotiq_client, pos=0.0, force=100, block=True)
        rospy.sleep(0.5)

    def open(self):
        action_name = 'command_robotiq_action'
        robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        robotiq_client.wait_for_server() # need to run roslaunch robotiq_2f_gripper_control robotiq_action_server.launch stroke:=0.140
        Robotiq.goto(robotiq_client, pos=0.140, force=100, block=True)
        rospy.sleep(0.5)
