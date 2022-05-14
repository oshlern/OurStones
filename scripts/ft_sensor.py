import numpy as np
import rospy
from geometry_msgs.msg import Wrench
from ati_ft_msgs.srv import FTSrv

class ForceTorqueSensor(object):
    def __init__(self):
        print("Initializing FT")
        self.wrench = None
        self.target_wrench_data = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        rospy.wait_for_service('get_ft_data')
        try:
            self.data_link = rospy.ServiceProxy('get_ft_data', FTSrv)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        print("Initialized FT")

    def read_force_torque(self, calibrated=True): # smoothing?
        self.wrench = self.data_link().wrench
        F = self.wrench.force
        T = self.wrench.torque
        # print("Raw FT Reading:\n", self.wrench)
        if calibrated:
            return np.array((F.x, F.y, F.z, T.x, T.y, T.z)) - self.target_wrench_data
        else:
            return np.array((F.x, F.y, F.z, T.x, T.y, T.z))

    def calibrate(self):
        self.target_wrench_data = self.read_force_torque(calibrated=False)
        return self.target_wrench_data
