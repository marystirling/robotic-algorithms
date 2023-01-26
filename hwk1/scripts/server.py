#!/usr/bin/env python

from __future__ import print_function

from hwk1.srv import AppendStrings, AppendStringsResponse
import rospy

def handle_append_two_strs(req):
    print("Returning [%s]"%(req.a + req.b))
    return AppendStringsResponse (req.a + req.b)

def append_two_strs_server():
    rospy.init_node("server")
    s = rospy.Service("append_strings", AppendStrings, handle_append_two_strs)
    print("Ready to append two strings")
    rospy.spin()

if __name__ == "__main__":
    append_two_strs_server()
