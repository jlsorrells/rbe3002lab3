#!/usr/bin/env python

import rospy
import sys
import tf
from lab3.srv import *
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, pi, trunc, ceil
from numpy import sign
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from rospy.client import wait_for_message
from nav_msgs.srv import *
from numpy.matlib import rand
import numpy
import random
import math
from nav_msgs.msg._GridCells import GridCells
from random import randint

def myclient(start, goal, map):
    #calls the A* service
    rospy.wait_for_service('Astar_pathfinding')
    try:
        myservice = rospy.ServiceProxy('Astar_pathfinding', FindPath)
        stuff = myservice(start, goal, map)
        return stuff.waypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
#===============================================================================
# def getmap():
#      global map
#      rospy.wait_for_service('dynamic_map')
#      try:
#          myservice = rospy.ServiceProxy('dynamic_map', GetMap)
#          stuff = myservice()
#          return stuff.map
#      except rospy.ServiceException, e:
#          print "Service call failed: %s"%e
#          return map
#===============================================================================
         
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    
    global pub
    global position
    global orientation
    global newpath
    
    #publish message to drive forward at requested speed
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    #pub.publish(twist)
    #print twist
    
    #continue until we have traveled the correct distance
    start_position = position
    while ((start_position[0]-position[0])**2+(start_position[1]-position[1])**2)**.5 < distance-.05 and not newpath:
        
        #start/stop more slowly
        #ramps up from speed/5 to speed over the first 20% of the distance, then down to 0 over the last 30%
        ratio = ((start_position[0]-position[0])**2+(start_position[1]-position[1])**2)**.5 / distance
        
        #dont actually ramp, because we're constantly redoing A* and getting new paths
        #which confuses the ramping
        ratio = 0.5
        
        if ratio < .2:
            #ramping up
            twist.linear.x = (1.0/5.0 + ratio*20.0/5.0) * speed
        elif ratio > .7:
            #ramping down
            twist.linear.x = (1.0 - (ratio - 0.7)/0.3) * speed
        else:
            #full speed
            twist.linear.x = speed
        pub.publish(twist)
        (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
        rospy.sleep(rospy.Duration(0, 1000000))
    
    #and then stop moving
    #===========================================================================
    # twist.linear.x = 0
    # twist.linear.y = 0
    # twist.linear.z = 0
    # twist.angular.x = 0
    # twist.angular.y = 0
    # twist.angular.z = 0
    #===========================================================================
    #pub.publish(twist)

#Accepts an angle and makes the robot rotate to it.
def rotate(angle):
    
    global pub
    global position
    global orientation
    global rotating
    rotating = True
    #publish message to rotate in the correct direction
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -sign(angle/180.0*pi - orientation)
    #pub.publish(twist)
    #print twist
    
    start_orientation = orientation
    
    #calculate desired orientation
    desired_orientation = angle/180.0*pi
    if desired_orientation > pi:
        desired_orientation -= 2*pi
    elif desired_orientation < -pi:
        desired_orientation += 2*pi
        
    distance = angle/180.0*pi - start_orientation
        
    offset = 0
    
    #continue until we are in the correct orientation 
    while abs(orientation - desired_orientation) > .1:
        
        #in case it went around -pi -> +pi
        if orientation > 0 and start_orientation < 0:
            offset = -2*pi
        elif orientation < 0 and start_orientation > 0:
            offset = 2*pi
            
        #start/stop more slowly
        #ramps up from 1/3 to 1 over the first 20% of the distance, then down over the last 30%
        ratio = (orientation - start_orientation + offset) / (distance)
        
        #so that it won't get stuck if it overshoots a bit
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
            
        #this turns off all the ramping stuff
        ratio = 0.5
            
        if ratio < .2:
            twist.angular.z = -(2.0/3.0 + ratio*1.0/3.0) * sign(distance)
        elif ratio > .7:
            twist.angular.z = -(1.0 - (ratio - 0.7)*13.3/3.0) * sign(distance)
        else:
            #twist.angular.z = -1#sign(distance)
            temp = orientation - desired_orientation
            if (temp >= 0 and temp <= pi) or (temp >= -2*pi and temp <= -1*pi):
                twist.angular.z = -1
                #print "rotate left"
                #print desired_orientation
            else:
                twist.angular.z = 1
                #print "rotate right"
                #print desired_orientation
        
        #if twist.angular.z < .05:
            #twist.angular.z = .05
        pub.publish(twist)
        (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
        rospy.sleep(rospy.Duration(0, 1000000))
    
    rotating = False
    #and then stop moving
    #===========================================================================
    # twist.linear.x = 0
    # twist.linear.y = 0
    # twist.linear.z = 0
    # twist.angular.x = 0
    # twist.angular.y = 0
    # twist.angular.z = 0
    #===========================================================================
    #pub.publish(twist)
    
#Odometry Callback function.
def read_odometry(msg):
#    print u"\n[x y \u03B8] = ".encode('utf-8')#unicode to print a theta symbol
#    print msg.pose.pose.position.x
#    print msg.pose.pose.position.y
#    print msg.pose.pose.orientation.z
    #save position to global var
    global position
    global orientation
    position = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    
    #angles are confusing
    orientation = atan2(2*(quat.y*quat.x+quat.w*quat.z),quat.w**2+quat.x**2-quat.y**2-quat.z**2)


def combineMaps2(localMap, globalMap):
        
    #print "combining maps"
    
    localMapWidth = localMap.info.width
    globalMapWidth = globalMap.info.width
    newGrid = OccupancyGrid()
    newGrid.header = globalMap.header
    newGrid.info = globalMap.info
    newGrid.data = globalMap.data
    
    #localMap.info.origin.position.y += localMapWidth * localMap.info.resolution / 2
    #print "local map origin, global = %s\n%s"%(localMap.info.origin.position,globalMap.info.origin.position)
    #print "local map resolution = %s"%(localMap.info.resolution)
    #determine the relative location of the maps
    if localMap.info.origin.position.x < globalMap.info.origin.position.x:
        tempX = 0
    else:
        tempX = (localMap.info.origin.position.x - globalMap.info.origin.position.x)/globalMap.info.resolution
    if localMap.info.origin.position.y < globalMap.info.origin.position.y:
        tempY = 0
    else:
        tempY = (localMap.info.origin.position.y - globalMap.info.origin.position.y)/globalMap.info.resolution
        
    n = tempY * globalMapWidth + tempX
    #print "x,y = %s,%s"%(tempX,tempY)
    #calculate upper bound
    if localMap.info.origin.position.y + localMapWidth*localMap.info.resolution > globalMap.info.origin.position.y + globalMapWidth*globalMap.info.resolution:
        U = localMap.info.origin.position.y/localMap.info.resolution + localMapWidth - globalMap.info.origin.position.y/globalMap.info.resolution - globalMapWidth
    else:
        U = 0
    #calculate left bound
    if localMap.info.origin.position.x < globalMap.info.origin.position.x:
        L = globalMap.info.origin.position.x/globalMap.info.resolution - localMap.info.origin.position.x/localMap.info.resolution
    else:
        L = 0
    #calculate right bound
    if localMap.info.origin.position.x + localMapWidth*localMap.info.resolution > globalMap.info.origin.position.x + globalMapWidth*globalMap.info.resolution:
        R = localMap.info.origin.position.x/localMap.info.resolution + localMapWidth - globalMap.info.origin.position.x/globalMap.info.resolution - globalMapWidth

    else:
        R = 0
    #calculate lower bound
    if localMap.info.origin.position.y < globalMap.info.origin.position.y:
        D = globalMap.info.origin.position.y/globalMap.info.resolution - localMap.info.origin.position.y/localMap.info.resolution
    else:
        D = 0
        
    #print "n,GWidth,LWitdh,g data = %s,%s,%s,%s"%(n, globalMapWidth, localMapWidth,len(globalMap.data))
    
    n2 = 0
    n3 = 0
    
    #make stuff ints
    #round to nearest number
    n = int(n+0.5)
    U = int(U+0.5)
    L = int(L+0.5)
    R = int(R+0.5)
    D = int(D+0.5)
    
    #print "U,L,R,D = %s,%s,%s,%s"%(U,L,R,D)
      
    #replace the use the value of the local map wherever the two maps overlap 
    for n2 in range(localMapWidth - U - D):
        for n3 in range(localMapWidth - L - R):
            tempGLoc = n + n2*globalMapWidth + n3
            tempLLoc = L + n2*localMapWidth + n3 + D * localMapWidth
            #if newGrid.data[tempGLoc] < localMap.data[tempLLoc]:
            #print "tempG,tempL = %s,%s"%(tempGLoc,tempLLoc)
            newGrid.data[tempGLoc] = localMap.data[tempLLoc]
    #print "done combining maps"
    return newGrid

#===============================================================================
# def combineMaps(localMap, globalMap):
#         
#     print "combining maps"
#     
#     localMapWidth = localMap.info.width
#     globalMapWidth = globalMap.info.width
#     newGrid = OccupancyGrid()
#     newGrid.header = globalMap.header
#     newGrid.info = globalMap.info
#     newGrid.data = globalMap.data
#     
#     #set n location
#     start = localMap.info.origin.position
#     start.x += (localMap.info.width / 2 + 1)* localMap.info.resolution
#     start.y += localMap.info.height / 2 * localMap.info.resolution
#     
#     start.x = -(int((start.x - globalMap.info.origin.position.x) / globalMap.info.resolution) - globalMap.info.width)
#     start.y = -int((-start.y + globalMap.info.origin.position.y) / globalMap.info.resolution)
#     n = start.y * globalMapWidth + start.x
#     print "x,y = %s,%s"%(start.x,start.y)
#     #calculate upper bound
#     if len(globalMap.data) - n <= globalMapWidth * localMapWidth/2:
#         U = trunc((len(globalMap.data) -n -1)/globalMapWidth)
#     else:
#         U = localMapWidth/2
#     #calculate left bound
#     if n%globalMapWidth < localMapWidth/2:
#         L = n%globalMapWidth
#     else:
#         L = localMapWidth/2
#     #calculate right bound
#     if globalMapWidth - n%globalMapWidth < localMapWidth/2:
#         R = globalMapWidth - n%globalMapWidth
#     else:
#         R = localMapWidth/2-1
#     #calculate lower bound
#     print "n,GWidth,LWitdh = %s,%s,%s"%(n, globalMapWidth, localMapWidth)
#     if n/globalMapWidth < localMapWidth/2:
#         D = trunc(n/globalMapWidth)
#     else:
#         D = localMapWidth/2-1
#     
#     n2 = 0
#     n3 = 0
#     
#     print "U,L,R,D = %s,%s,%s,%s"%(U,L,R,D)
#        
#     for n2 in range(int(U+D+1)):
#         for n3 in range(int(L+R+1)):
#             tempGLoc = int(n + globalMapWidth * (n2 - U) + (n3 - L))
#             tempLLoc = int((localMapWidth/2-L) + n3 + (L+R+1)*n2)
#             #if newGrid.data[tempGLoc] < localMap.data[tempLLoc]:
#             print "tempG,tempL = %s,%s"%(tempGLoc,tempLLoc)
#             print "size of local map = %s"%(len(localMap.data))
#             print "U,L,R,D = %s,%s,%s,%s"%(U,L,R,D)
#             print "\n"
#             newGrid.data[tempGLoc] = localMap.data[tempLLoc]
# 
#     return newGrid
#===============================================================================

def expandObstacle(inputMap):
    
    robotRadius = .25#amount to expand obstacles by
    expandedObsVal = 0.05#how sure we are that things near obstacles are also obstacles
    newGrid = OccupancyGrid()
    newGrid.data = list(inputMap.data)
    newGrid.header = inputMap.header
    newGrid.info = inputMap.info
    inputArray = inputMap.data
    gridResolution = newGrid.info.resolution
    gridWidth = newGrid.info.width
    
    n = 0
    #print "expanding obstacles"
    numExpansions = ceil(robotRadius/gridResolution)
    
    #for each cell, increase the value of nearby cells proportional to this cell's value
    for n in range(len(inputArray)):
            
        if inputArray[n] > 0:
            #determine L bound
            if n%gridWidth < numExpansions:
                L = n%gridWidth
            else:
                L = numExpansions
            #determine Upper bound
            if n/gridWidth < numExpansions:
                D = trunc(n/gridWidth)
            else:
                D = numExpansions        
            #determine R bound
            if gridWidth - n%gridWidth - 1 < numExpansions:
                R = gridWidth - n%gridWidth - 1
            else:
                R = numExpansions
            #determine depth
            if len(inputArray) - n <= numExpansions * gridWidth:
                U = trunc((len(inputArray) - n - 1)/gridWidth)
            else:
                U = numExpansions
        
            n2 = 0
            n3 = 0
        
            for n2 in range(int(U+D+1)):
                for n3 in range(int(L+R+1)):
                    #print "girdwidth %s, U %s, L %s, D %s, R %s"%(gridWidth,U,L,D,R)
                    tempLoc = int(n + gridWidth * (U - n2) + (n3 - L))
                    #print "n = %s, temploc = %s (%s, %s)"%(n, tempLoc, n3, n2)
                    newGrid.data[tempLoc] += inputArray[n] * expandedObsVal
                    if newGrid.data[tempLoc] > 100:
                        newGrid.data[tempLoc] = 100

    return newGrid

#this reads local maps from gmapping
def read_map(msg):
    global map 
    global arrayTravelled
    global arrayFrontier
    global position
    global goals
    global pathpoints
    global newpath
    global rotating

    #because apparently it isn't a list already?
    msg.data = list(msg.data)
    
    #newpath = True
    #print "asking for a new map"
    #save the most recent map
    
    temp_map = msg
    #print "got a new map"
    #print map
    if map == 0:
        #=======================================================================
        # print "requesting global map from gmapping"
        # map = getmap() 
        # print "finally got a global map from gmapping"
        #=======================================================================
        map = msg
        map.info.height = 26
        map.info.width = 26
        
    #add this local map into our combined map
    map = combineMaps2(expandObstacle(temp_map), map)

    mapresolution = map.info.resolution
    start = Point(position[0], position[1], 0)
    
    #redo A* since we got a new map
    pathpoints, newpath = myclient(start, goals[0], map), True

    #print "current location is (%s, %s)"%(start.x, start.y)
    if not pathpoints:
        #try the next goal if the current one is now unreachable
        print "unable to reach goal (%s, %s)"%(goals[0].x, goals[0].y)
        set_next_goal()
        
    #===========================================================================
    # while rotating:
    #     rospy.sleep(rospy.Duration(0.05))
    #===========================================================================
     
    #wait a bit so we don't just constantly replan if gmapping starts updating quickly  
    rospy.sleep(rospy.Duration(4.0))
        
    #print "returning from callback"
    
#this reads the /map from gmapping
def read_global_map(msg):
    global map
    #forget about the previous local maps and use this global map
    map = expandObstacle(msg)
    
def read_new_goals(msg):
    global goals
    
    #get the list of positions
    temp = msg.cells
    
    #sort by closeness to robot
    (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    temp.sort(key=lambda g: ((g.x-position[0])**2 + (g.y-position[1])**2)**.5, reverse=False)
    #temp.sort(key=lambda g: randint(0,100), reverse=True)
    #save in global variable if we're done with the current goals
    #otherwise, just ignore these and keep doing the previous goals
    if len(goals) <= 1:
        goals = temp
        
    print "got new goals"
    print goals
    
def set_next_goal():
    global goals
    
    try:
        (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            
        if len(goals) < 2:
            #drive randomly if no more goals
            print "out of goals, moving randomly"
            goals.append(Point(position[0] + randint(-20,20)/10.0, position[1] + randint(-20,20)/10.0,0))
            print goals
            start = Point(position[0], position[1], 0)
            pathpoints, newpath = myclient(start, goals[0], map), True
            if not pathpoints:
                print "unable to reach goal (%s, %s)"%(goals[0].x, goals[0].y)
                set_next_goal()
            return
        
        #remove the top goal
        goals.pop(0)
        goals.sort(key=lambda g: ((g.x-position[0])**2 + (g.y-position[1])**2)**.5, reverse=False)
        #goals.sort(key=lambda g: randint(0,100), reverse=True)
        
        if set_next_goal.start == 0:
            set_next_goal.start = rospy.get_time()
        
        #use random goals for the first 5 minutes
        if (rospy.get_time() - set_next_goal.start) < 5*60:
            goals = []
            print "early, using random goals"
            goals.append(Point(position[0] + randint(-20,20)/10.0, position[1] + randint(-20,20)/10.0,0))
        
        print "moving on to next goal"
        #print goals[0]
        
        #redo A* since we got a new goal
        start = Point(position[0], position[1], 0)
        pathpoints, newpath = myclient(start, goals[0], map), True
        if not pathpoints:
            #if this goal is unreachable, go to the next one
            print "unable to reach goal (%s, %s)"%(goals[0].x, goals[0].y)
            set_next_goal()
    except IndexError:
        #just go to a random point if something is broken
        goals.append(Point(position[0] + randint(-20,20)/10.0, position[1] + randint(-20,20)/10.0,0))
        
    
set_next_goal.start = 0    

    
if __name__ == "__main__":
    
    global pub
    global map_pub
    global map_sub
    global map
    global goals
    global pathpoints
    global newpath
    global position
    global rotating
    rotating = False
    map = 0
    
    rospy.init_node('barth_sorrells_wu_lab4_node')
    #===========================================================================
    # 
    # if len(sys.argv) == 3:
    #     #start = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
    #     goal = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
    # else:
    #     sys.exit(1)
    #===========================================================================
    goals = []
    goals.append(Point(0,0,0))
        
    pathpoints = []
    listener = tf.TransformListener()
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    #odom_sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    
    rospy.sleep(rospy.Duration(2.0))
    
    (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
        
    rospy.sleep(rospy.Duration(1.0))
    
    #rospy.Timer(rospy.Duration(1.0), read_map)
    map_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
    #map_sub2 = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_global_map, queue_size=1)
    map_sub2 = rospy.Subscriber('/map', OccupancyGrid, read_global_map, queue_size=1)
    map_pub = rospy.Publisher('/map2', OccupancyGrid)
    goal_sub = rospy.Subscriber('/final_project/goals', GridCells, read_new_goals, queue_size=1)
        
    rospy.sleep(rospy.Duration(1.0))
    
    #print "Requesting path to (%s,%s)"%(goal.x, goal.y)
    
    startTime = rospy.get_time()
    
    newpath = False    
    while not rospy.is_shutdown():
        
        #don't go to another goal if after 19 minutes
        if (rospy.get_time() - startTime) > 19*60:
            print "\n\nDone generating map.\n\n"
            break
        
        (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
        
        j = Point(position[0],position[1],0)
        #if we have a path, follow it
        if pathpoints and len(pathpoints) > 0:
            for i in xrange(1,len(pathpoints)):
                distance = ((pathpoints[i].x-j.x)**2+(pathpoints[i].y-j.y)**2)**.5
                angle = atan2(pathpoints[i].y - j.y, pathpoints[i].x - j.x)*180.0/pi
                #print "driving from (%s,%s) to (%s,%s)"%(j.x,j.y,pathpoints[i].x,pathpoints[i].y)
                #print "turning to angle %s degrees, then driving %s meters"%(angle, distance)
                #turn to the correct direction
                rotate(angle)
                #then drive the distance
                driveStraight(0.2, distance)
                #save previous
                (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
                j = Point(position[0], position[1], 0)
                
                if newpath:
                    break
            if len(goals) > 0 and abs(position[0] - goals[0].x) < .2 and abs(position[1] - goals[0].y) < .2:
                #if we're here, spin around, then go to the next place
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = -1
                for k in range(50):
                    pub.publish(twist)
                    rospy.sleep(rospy.Duration(0.1))
                set_next_goal()
        #if we don't have a path, just spin around
        else:
            #print "spinning"
            rotate(orientation*180.0/pi+90)
            #print "done spinning"
            #print newpath
        #wait for a new path
        while newpath == False and not rospy.is_shutdown():
            #print "spinning"
            rotate(orientation*180.0/pi+90)
        newpath = False
    
    
  
        
