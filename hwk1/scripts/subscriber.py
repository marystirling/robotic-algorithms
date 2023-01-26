#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "Received: %s", data.data)
    print("Received: " + str(data.data))

def listener():
    rospy.init_node('subscriber', anonymous = True)
    rospy.Subscriber("robots", Int64, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
