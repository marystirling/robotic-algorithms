#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from hwk1.srv import *

def append_strings_client(a, b):
    rospy.wait_for_service('append_strings')
    try:
        appended = rospy.ServiceProxy('append_strings', AppendStrings)
        resp1 = appended(a, b)
        return str(resp1).split(": ")[-1]
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [a,b]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        a = str(sys.argv[1])
        b = str(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting to append %s and %s"%(a,b))
    print("Received: %s"%(append_strings_client(a, b)))
