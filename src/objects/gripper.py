
import rospy
import _ as robot_gripper # TODO

class Gripper(Object):
    def __init__(self):
        self.gripper = robot_gripper.Gripper('right')

    def calibrate(self):
        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating Gripper...')
        self.gripper.calibrate()
        rospy.sleep(1.0)
        self.close()
        self.open()
        print('Done!')
    
    def close(self):
        # gripper.close(block=True)
        self.gripper.close()
        rospy.sleep(0.5)

    def open(self):
        # gripper.open(block=True)
        gripper.open()
        rospy.sleep(0.5)
