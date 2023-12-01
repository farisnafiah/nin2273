#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import random

def AscanTalker():
    pub = rospy.Publisher('Thickness1', Float64, queue_size=10)
    rospy.init_node('AscanTalker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        fake_thickness = 15.5 + round(random.uniform(-0.1,0.1), 2)
        pub.publish(fake_thickness)
        rate.sleep()

if __name__ == '__main__':
    try:
        AscanTalker()
    except rospy.ROSInterruptException:
        pass
