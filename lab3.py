#!/usr/bin/env python

import rospy, tf
from geometry_msgs.msg._PoseStamped import PoseStamped
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from nav_msgs.msg._GridCells import GridCells
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from numpy import sign, ndarray
from visualization_msgs.msg._Marker import Marker

#TODO
#make launch file to start map_server and this and rviz at the same time
#somehow make rviz's grid line up with the map (its working now??)
#send robot model to rviz

class point:
    def __init__(self, x1, y1, Parent, Cost):
        self.x = x1
        self.y = y1
        self.parent = Parent
        self.cost = Cost
    
    def equals(self, point2):
        if self.x == point2.x and self.y == point2.y:
            return True
        else:
            return False
            
    def getCost(self):
        global arrayTravelled
        global start
        
        cost = getHeuristics(self.x, self.y)

        if self.parent.x == self.x or self.parent.y == self.y:
            cost += 1
        else:
            #diagonal moves are longer
            cost += 1.414
            
        cost += self.parent.cost - getHeuristics(self.parent.x, self.parent.y)
        
        return cost
    
    def getPath(self):
        global arrayTravelled
        global start
        
        result = []
        
        tempPoint = point(self.x, self.y, self.parent, self.cost)
        arrayTravelled.remove(self)
        while tempPoint.x != start.x or tempPoint.y != start.y:
            result.append(tempPoint)
            tempPoint = tempPoint.parent
            arrayTravelled.remove(tempPoint)
        result.append(tempPoint)
        displayPath(result)
        displayWaypoints(pathWaypoints(result))
    

def createArray(inputPoints):
    global matrixPoints
    
    matrixPoints = ndarray((map.info.width,map.info.height))
    
    originalIndex = 0
    for y in range(map.info.height):
        for x in range(map.info.width):
            matrixPoints[x][map.info.height - y - 1] = inputPoints[originalIndex]
            originalIndex += 1

def aStar(startnode):
    global matrixPoints
    global arrayTravelled
    global arrayFrontier
    global map
    
    arrayTravelled = []
    arrayFrontier = [startnode]
    createArray(map.data)
    
    freeSpaceValue = 50
    
    while len(arrayFrontier) != 0:
        
        node = arrayFrontier[0]
    
        x1 = node.x
        y1 = node.y
        
        #add point to travelled
        arrayTravelled.append(node)
        arrayFrontier.pop(0)
        
        if x1 == goal.x and y1 == goal.y:
            return node  
        
        #UL
        if matrixPoints[x1-1][y1-1]<freeSpaceValue:
            frontierAdd(x1-1, y1-1, node)
        #UM
        if matrixPoints[x1][y1-1]<freeSpaceValue:
            frontierAdd(x1, y1-1, node)
        #UR
        if matrixPoints[x1+1][y1-1]<freeSpaceValue:
            frontierAdd(x1+1, y1-1, node)
        #RM
        if matrixPoints[x1+1][y1]<freeSpaceValue:
            frontierAdd(x1+1, y1, node)
        #RB
        if matrixPoints[x1+1][y1+1]<freeSpaceValue:
            frontierAdd(x1+1, y1+1, node)
        #BM
        if matrixPoints[x1][y1+1]<freeSpaceValue:
            frontierAdd(x1, y1+1, node)
        #BL
        if matrixPoints[x1-1][y1+1]<freeSpaceValue:
            frontierAdd(x1-1, y1+1, node)
        #LM
        if matrixPoints[x1-1][y1]<freeSpaceValue:
            frontierAdd(x1-1, y1, node)
            
        displayProgress(arrayTravelled, arrayFrontier)
          
        #rospy.sleep(rospy.Duration(.05, 0))
    
    #return 0 if no path is found
    rospy.logwarn("no path found from (%d, %d) to (%d, %d)", 
                 start.x, start.y, goal.x, goal.y)
    return 0
        
    

def getHeuristics(x1, y1):
    #heuristic = max(abs(x1-goal.x), abs(y1-goal.y))
    heuristic = ((x1-goal.x)**2+(y1-goal.y)**2)**.5
    return heuristic
    
def frontierAdd(x1, y1, node):
    global arrayTravelled
    global arrayFrontier

    
    newPoint = point(x1, y1, node, 0)
    newPoint.cost = newPoint.getCost()
    
    #check if point is already in frontier
    for p in arrayFrontier:
        if p.equals(newPoint):
            if p.cost > newPoint.cost:
                arrayFrontier.remove(p)
                break
            else:
                return
        
    #check if point is already explored
    for p in arrayTravelled:
        if p.equals(newPoint):
            return
    
    
    n = 0
    for node2 in arrayFrontier:
        if newPoint.cost > node2.cost:
            n += 1
        else:
            break
       
    arrayFrontier.insert(n, newPoint)

def read_map(msg):
    global map 
    global arrayTravelled
    global arrayFrontier
    
    #save the most recent map
    map = msg 
    
    displayPath([])
    displayWaypoints([])
    result = aStar(point(start.x, start.y, 0, 0))
    rospy.sleep(rospy.Duration(.05, 0))
    if result != 0:
        result.getPath()
    displayProgress(arrayTravelled, arrayFrontier)
    
    #print msg
    #print "\n"
    
def read_goal(msg):
    global goal
    global map
    
    goal = msg.pose.position
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose = msg.pose
    marker.text = "G"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lab3markers"
    marker.id = 0
    marker.scale.x = map.info.resolution
    marker.scale.y = map.info.resolution
    marker.scale.z = map.info.resolution
    marker.color.r = 1
    marker.color.a = 1
    marker.action = Marker.ADD
    marker_pub.publish(marker)
    
    #convert to grid cell
    goal.x = int(goal.x/map.info.resolution + 0.5*sign(goal.x))
    goal.y = int(goal.y/map.info.resolution + 0.5*sign(goal.y))
    #convert so that upper left is 0,0
    goal.x = goal.x + map.info.width/2
    goal.y = -goal.y + map.info.height/2
    #print goal
    
    displayPath([])
    displayWaypoints([])
    result = aStar(point(start.x, start.y, 0, 0))
    rospy.sleep(rospy.Duration(.05, 0))
    if result != 0:
        result.getPath()
    displayProgress(arrayTravelled, arrayFrontier)
    
    
def read_start(msg):
    global start
    global map
    
    start = msg.pose.pose.position
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose = msg.pose.pose
    marker.text = "S"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lab3markers"
    marker.id = 1
    marker.scale.x = map.info.resolution
    marker.scale.y = map.info.resolution
    marker.scale.z = map.info.resolution
    marker.color.r = 1
    marker.color.a = 1
    marker.action = Marker.ADD
    marker_pub.publish(marker)
    
    #convert to grid cell
    start.x = int(start.x/map.info.resolution + 0.5*sign(start.x))
    start.y = int(start.y/map.info.resolution + 0.5*sign(start.y))
    #convert so that upper left is 0,0
    start.x = start.x + map.info.width/2
    start.y = -start.y + map.info.height/2
    #print start
    
    displayPath([])
    displayWaypoints([])
    result = aStar(point(start.x, start.y, 0, 0))
    rospy.sleep(rospy.Duration(.05, 0))
    if result != 0:
        result.getPath()
    displayProgress(arrayTravelled, arrayFrontier)
    
def displayProgress(e_array, f_array):
    global explored_pub
    global frontier_pub
    global map
    
    cells = GridCells()
    cells.cell_height = map.info.resolution
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #publish explored cells
    array = []
    for i in e_array:
        x = (i.x - map.info.width/2)*map.info.resolution
        y = (-i.y + map.info.height/2)*map.info.resolution
        array.append(Point(x,y,0))
    cells.cells = array
    explored_pub.publish(cells)
    
    #publish frontier cells
    array = []
    for i in f_array:
        x = (i.x - map.info.width/2)*map.info.resolution
        y = (-i.y + map.info.height/2)*map.info.resolution
        array.append(Point(x,y,0))
    cells.cells = array
    frontier_pub.publish(cells)
    
def displayPath(p_array):
    global path_pub
    global map
    global arrayTravelled
    
    cells = GridCells()
    cells.cell_height = map.info.resolution
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #publish path cells
    array = []
    for i in p_array:
        x = (i.x - map.info.width/2)*map.info.resolution
        y = (-i.y + map.info.height/2)*map.info.resolution
        array.append(Point(x,y,0))
    cells.cells = array
    path_pub.publish(cells)


def displayWaypoints(w_array):
    global marker_pub
    global map
    
    #delete old markers
    for i in range(2,2+displayWaypoints.prevWaypoints):
        marker = Marker()
        marker.ns = "lab3markers"
        marker.id = i
        marker.action = Marker.DELETE
        marker.header.frame_id = "map"
        marker.type = Marker.ARROW
        marker_pub.publish(marker)
    
    for i in range(1,len(w_array)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.ARROW
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lab3markers"
        marker.id = 2+i-1
        marker.color.r = 1
        marker.color.a = 1
        marker.action = Marker.ADD
        marker.scale.x = map.info.resolution/3
        marker.scale.y = 2*map.info.resolution/3
        marker.points = []
        x1 = (w_array[i-1].x - map.info.width/2)*map.info.resolution
        y1 = (-w_array[i-1].y + map.info.height/2)*map.info.resolution
        x2 = (w_array[i].x - map.info.width/2)*map.info.resolution
        y2 = (-w_array[i].y + map.info.height/2)*map.info.resolution
        marker.points.append(Point(x1, y1, 0))
        marker.points.append(Point(x2, y2, 0))
        marker_pub.publish(marker)
        displayWaypoints.prevWaypoints = i
         
#I guess this is how to make a static variable in python
displayWaypoints.prevWaypoints = 0

def pathWaypoints(path):
    endpoints = []
    previous_x_velocity = 0
    previous_y_velocity = 0
    for i in path[:-1]:
        current_x_velocity = i.x - i.parent.x
	current_y_velocity = i.y - i.parent.y
	if current_x_velocity != previous_x_velocity or current_y_velocity != previous_y_velocity: 
	    endpoints.append(i)
	    #print (i.x,i.y)
	    previous_x_velocity = current_x_velocity
	    previous_y_velocity = current_y_velocity
    endpoints.append(path[-1])
    #print (path[-1].x, path[-1].y)
    endpoints.reverse()
    return endpoints
    

# This is the program's main function
if __name__ == '__main__':
    global map
    global marker_pub
    global path_pub
    global explored_pub
    global frontier_pub
    
    global start
    global goal
    start = Point(3,30,0)
    goal = Point(10,30,0)
    
    # Change this node name to include your username
    rospy.init_node('Barth_Sorrells_Wu_Lab_3_node')
    
    path_pub = rospy.Publisher('/lab3/pathcells', GridCells)
    explored_pub = rospy.Publisher('/lab3/exploredcells', GridCells)
    frontier_pub = rospy.Publisher('/lab3/frontiercells', GridCells)
    marker_pub = rospy.Publisher('/lab3/markers', Marker)
    
    move_base_simple_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, read_goal, queue_size=1)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)
    initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, read_start, queue_size=1)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    
    print "starting lab 3"
    
    # wait for events
    rospy.spin()
    
    print "\nending lab 3"


