#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

def publishing():
    pub = rospy.Publisher('robots', Int64, queue_size = 10)
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        num = 891
        rospy.loginfo("Sending %d", num)
        pub.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        publishing()
    except rospy.ROSInterruptException:
        pass

