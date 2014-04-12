#!/usr/bin/env python

import rospy
import sys
from lab3.srv import *
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, pi
from numpy import sign

def myclient(start, goal):
    rospy.wait_for_service('Astar_pathfinding')
    try:
        myservice = rospy.ServiceProxy('Astar_pathfinding', FindPath)
        stuff = myservice(start, goal)
        return stuff.waypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    
    global pub
    global position
    global orientation
    
    #publish message to drive forward at requested speed
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    #pub.publish(twist)
    print twist
    
    #continue until we have traveled the correct distance
    start_position = position
    while ((start_position.x-position.x)**2+(start_position.y-position.y)**2)**.5 < distance-.05:
        
        #start/stop more slowly
        #ramps up from speed/5 to speed over the first 20% of the distance, then down to 0 over the last 30%
        ratio = ((start_position.x-position.x)**2+(start_position.y-position.y)**2)**.5 / distance
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
        rospy.sleep(rospy.Duration(0, 1000000))
    
    #and then stop moving
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    
    global pub
    global position
    global orientation
    
    #publish message to rotate in the correct direction
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -sign(angle/180.0*pi - start_orientation)
    #pub.publish(twist)
    print twist
    
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
    while abs(orientation - desired_orientation) > .05:
        
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
            
        if ratio < .2:
            twist.angular.z = -(1.0/3.0 + ratio*10.0/3.0) * sign(distance)
        elif ratio > .7:
            twist.angular.z = -(1.0 - (ratio - 0.7)*13.3/3.0) * sign(distance)
        else:
            twist.angular.z = -sign(distance)
            
        if twist.angular.z < .05:
            twist.angular.z = .05
        pub.publish(twist)
        rospy.sleep(rospy.Duration(0, 1000000))
    
    #and then stop moving
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    
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

if __name__ == "__main__":
    
    global pub
    
    rospy.init_node('barth_sorrells_wu_lab4_node')
    
    if len(sys.argv) == 5:
        start = Point(float(sys.argv[1]), float(sys.argv[2]), float(0))
        goal = Point(float(sys.argv[3]), float(sys.argv[4]), float(0))
    else:
        sys.exit(1)
        
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    odom_sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
        
    rospy.sleep(rospy.Duration(1.0))
        
    print "Requesting path from (%s,%s) to (%s,%s)"%(start.x, start.y, goal.x, goal.y)
    path = myclient(start, goal)
    if not path:
        print "unable to get path"
        sys.exit(0)
    
    #TODO, fix this
    mapresolution = .2
    
    j = Point(start.x,start.y,0)
    for i in path[1:]:
        #calculate needed motions
        distance = mapresolution*((i.x-j.x)**2+(i.y-j.y)**2)**.5
        angle = atan2(i.y - j.y, i.x - j.x)*180.0/pi
        print "driving from (%s,%s) to (%s,%s)"%(j.x,j.y,i.x,i.y)
        print "turning to angle %s degrees, then driving %s meters"%(angle, distance)
        #turn to the correct direction
        rotate(angle)
        #then drive the distance
        driveStraight(0.5, distance)
        #save previous
        j = i
        
