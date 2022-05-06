import rospy
from geometry_msgs.msg import Wrench

class FT:
    data = None
    # def __init__(self):
    #     self.data = None
    # def update(self, data):
    #     self.data = data

#def get_data():
 #   return data

def cb(wrench):
 #   global data
    FT.data = wrench
# = wrench

if __name__ == "__main__":
    #data = None
    rospy.init_node('ft_sub')
    sub = rospy.Subscriber('/ft', Wrench, cb)
    rospy.spin()
    for i in range(100):
        print(FT.data)
        rospy.sleep(0.5)
