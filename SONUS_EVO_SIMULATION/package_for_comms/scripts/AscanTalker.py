#!/usr/bin/env python3
# license removed for brevity
import rospy
from package_for_comms_msgs.msg import SonusEvoData

def AscanTalker():
    pub = rospy.Publisher('Thickness1', SonusEvoData, queue_size=10)
    rospy.init_node('AscanTalker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        fake_thickness = 9.8
        msg = SonusEvoData()
        msg.data = fake_thickness
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        AscanTalker()
    except rospy.ROSInterruptException:
        pass
