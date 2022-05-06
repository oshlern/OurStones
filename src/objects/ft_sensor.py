import numpy as np
import rospy

class ForceTorqueSensor(Object):
    def __init__(self):
        self.wrench = None
        rospy.init_node('ft_sub')
        sub = rospy.Subscriber('/ft', Wrench, self.update_data)
        rospy.spin()
    
    def update_data(self, wrench):
        self.wrench = wrench

    def read_force_torque(self): # smoothing?
        return self.wrench