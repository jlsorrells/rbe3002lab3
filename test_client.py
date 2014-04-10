#!/usr/bin/env python

import rospy
import sys
from lab3.srv import *
from geometry_msgs.msg._Point import Point

def myclient(start, goal):
    rospy.wait_for_service('Astar_pathfinding')
    try:
        myservice = rospy.ServiceProxy('Astar_pathfinding', FindPath)
        stuff = myservice(start, goal)
        return stuff.waypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    if len(sys.argv) == 5:
        start = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
        goal = Point(float(sys.argv[3]), float(sys.argv[4]), float(0))
    else:
        sys.exit(1)
    print "Requesting path from (%s,%s) to (%s,%s)"%(start.x, start.y, goal.x, goal.y)
    try:
        for i in myclient(start, goal):
            print (i.x, i.y)
    except:
        print "unable to get path"
