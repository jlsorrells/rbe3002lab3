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
#     global map
#     rospy.wait_for_service('dynamic_map')
#     try:
#         myservice = rospy.ServiceProxy('dynamic_map', GetMap)
#         stuff = myservice()
#         return stuff.map
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
#         return map
#         
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
        
        #dont actually ramp
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
    
#Accepts an angle and makes the robot rotate around it.
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
            
        #this stops all the ramping stuff
        ratio = 0.5
            
        if ratio < .2:
            twist.angular.z = -(2.0/3.0 + ratio*1.0/3.0) * sign(distance)
        elif ratio > .7:
            twist.angular.z = -(1.0 - (ratio - 0.7)*13.3/3.0) * sign(distance)
        else:
            #twist.angular.z = -1#sign(distance)
            if orientation > desired_orientation:
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

def combineMaps(localMap, globalMap):
        
    print "combining maps"
    
    localMapWidth = localMap.info.width
    globalMapWidth = globalMap.info.width
    newGrid = OccupancyGrid()
    newGrid.header = globalMap.header
    newGrid.info = globalMap.info
    newGrid.data = globalMap.data
    #set n location
    start = Point(position[0], position[1], 0)
    start.x = -(int((start.x - globalMap.info.origin.position.x) / globalMap.info.resolution) - globalMap.info.width)
    start.y = -int((-start.y + globalMap.info.origin.position.y) / globalMap.info.resolution)
    n = start.y * globalMapWidth + start.x
    print "x,y = %s,%s"%(start.x,start.y)
    #calculate upper bound
    if len(globalMap.data) - n <= globalMapWidth * localMapWidth/2:
        U = trunc((len(globalMap.data) -n -1)/globalMapWidth)
    else:
        U = localMapWidth/2
    #calculate left bound
    if n%globalMapWidth < localMapWidth/2:
        L = n%globalMapWidth
    else:
        L = localMapWidth/2
    #calculate right bound
    if globalMapWidth - n%globalMapWidth < localMapWidth/2:
        R = globalMapWidth - n%globalMapWidth
    else:
        R = localMapWidth/2
    #calculate lower bound
    print "n,GWidth,LWitdh = %s,%s,%s"%(n, globalMapWidth, localMapWidth)
    if n/globalMapWidth < localMapWidth/2:
        D = trunc(n/globalMapWidth)
    else:
        D = localMapWidth/2
    
    n2 = 0
    n3 = 0
    
    print "U,L,R,D = %s,%s,%s,%s"%(U,L,R,D)
       
    for n2 in range(int(U+D+1)):
        for n3 in range(int(L+R+1)):
            tempGLoc = int(n + globalMapWidth * (n2 - U) + (n3 - L))
            tempLLoc = int((localMapWidth/2-L) + n3 + (L+R+1)*n2)
            #if newGrid.data[tempGLoc] < localMap.data[tempLLoc]:
            print "tempG,tempL = %s,%s"%(tempGLoc,tempLLoc)
            newGrid.data[tempGLoc] = localMap.data[tempLLoc]

    return newGrid

def expandObstacle(inputMap):
    
    robotRadius = .07#amount to expand obstacles by
    expandedObsVal = 0.2#how sure we are that things near obstacles are also obstacles
    newGrid = OccupancyGrid()
    newGrid.data = list(inputMap.data)
    newGrid.header = inputMap.header
    newGrid.info = inputMap.info
    inputArray = inputMap.data
    gridResolution = newGrid.info.resolution
    gridWidth = newGrid.info.width
    
    n = 0
    print "expanding obstacles"
    numExpansions = ceil(robotRadius/gridResolution)
    
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
    global map_pub
    #for i in xrange(10000):
        #map_pub.publish(newGrid)
    return newGrid

def read_map(msg):
    global map 
    global arrayTravelled
    global arrayFrontier
    global position
    global goal
    global pathpoints
    global newpath
    global rotating

    #because apperently it isn't a list already?
    msg.data = list(msg.data)
    
    #newpath = True
    #print "asking for a new map"
    #save the most recent map
    #temp_map = getmap() 
    temp_map = msg
    print "got a new map"
    #print map
    if map == 0:
        map = msg
        
    map = combineMaps(expandObstacle(temp_map), map)

    mapresolution = map.info.resolution
    start = Point(position[0], position[1], 0)
    
    pathpoints, newpath = myclient(start, goal, map), True

    #print "current location is (%s, %s)"%(start.x, start.y)
    if not pathpoints:
        print "unable to reach goal (%s, %s)"%(goal.x, goal.y)
        
    while rotating:
        rospy.sleep(rospy.Duration(0.05))
     
    #wiat a bit so we aren't just constantly replanning   
    rospy.sleep(rospy.Duration(4.0))
        
    print "returning from callback"
    
def read_global_map(msg):
    global map
    map = expandObstacle(msg)
    

if __name__ == "__main__":
    
    global pub
    global map_pub
    global map_sub
    global map
    global goal
    global pathpoints
    global newpath
    global position
    global rotating
    rotating = False
    map = 0
    
    rospy.init_node('barth_sorrells_wu_lab4_node')
    
    if len(sys.argv) == 3:
        #start = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
        goal = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
    else:
        sys.exit(1)
        
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
    map_sub2 = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_global_map, queue_size=1)
    map_pub = rospy.Publisher('/map2', OccupancyGrid)
        
    rospy.sleep(rospy.Duration(1.0))
    
    #print "Requesting path to (%s,%s)"%(goal.x, goal.y)
    
    newpath = False    
    while not rospy.is_shutdown():
        (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
        
        j = Point(position[0],position[1],0)
        if pathpoints and len(pathpoints) > 0:
            for i in xrange(1,len(pathpoints)):
                distance = ((pathpoints[i].x-j.x)**2+(pathpoints[i].y-j.y)**2)**.5
                angle = atan2(pathpoints[i].y - j.y, pathpoints[i].x - j.x)*180.0/pi
                #print "driving from (%s,%s) to (%s,%s)"%(j.x,j.y,pathpoints[i].x,pathpoints[i].y)
                #print "turning to angle %s degrees, then driving %s meters"%(angle, distance)
                #turn to the correct direction
                rotate(angle)
                #then drive the distance
                driveStraight(0.1, distance)
                #save previous
                (position, quat) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                orientation = atan2(2*(quat[1]*quat[0]+quat[3]*quat[2]),quat[3]**2+quat[0]**2-quat[1]**2-quat[2]**2)
                j = Point(position[0], position[1], 0)
                
                if newpath:
                    break
        else:
            print "spinning"
            rotate(orientation*180.0/pi+90)
            print "done spinning"
            print newpath
        while newpath == False:
            pass
        newpath = False
    
    
  
        
