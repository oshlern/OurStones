#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench
from ati_ft_msgs.srv import FTSrv

class FT:
    data = None

def cb(wrench):
    FT.data = wrench

def serve(req):
    return FT.data

if __name__ == "__main__":
    rospy.init_node('ft_sub')
    sub = rospy.Subscriber('/ft', Wrench, cb)
    svc = rospy.Service('get_ft_data', FTSrv, serve)
    rospy.spin()
