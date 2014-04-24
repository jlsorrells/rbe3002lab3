#!/usr/bin/env python

import rospy
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from nav_msgs.msg._GridCells import GridCells
from geometry_msgs.msg._Point import Point
from math import trunc

def read_map(msg):
    global goal_pub
    
    mapResolution = msg.info.resolution
    mapWidth = msg.info.width
    inputArray = msg.data
    result = GridCells();
    
    n = 0
    #print "expanding obstacles"
    print "searching for new frontier cells"
    numExpansions = 1
    freeSpaceValue = 50
    
    for n in range(len(inputArray)):
        
        #look at unknown cells
        if inputArray[n] == -1:
            #determine L bound
            if n%mapWidth < numExpansions:
                L = n%mapWidth
            else:
                L = numExpansions
            #determine Upper bound
            if n/mapWidth < numExpansions:
                D = trunc(n/mapWidth)
            else:
                D = numExpansions        
            #determine R bound
            if mapWidth - n%mapWidth - 1 < numExpansions:
                R = mapWidth - n%mapWidth - 1
            else:
                R = numExpansions
            #determine depth
            if len(inputArray) - n <= numExpansions * mapWidth:
                U = trunc((len(inputArray) - n - 1)/mapWidth)
            else:
                U = numExpansions
        
            n2 = 0
            n3 = 0
        
            stop = 0
            for n2 in range(int(U+D+1)):
                for n3 in range(int(L+R+1)):
                    tempLoc = int(n + mapWidth * (U - n2) + (n3 - L))
                    #check if this unknown cell is next to free space
                    if inputArray[tempLoc] != -1 and inputArray[tempLoc] < freeSpaceValue:
                        result.cells.append(Point(n%mapWidth, int(n/mapWidth), 0))
                        stop = 1
                        break
                if stop == 1:
                    break
    
    #convert to position
    for i in result.cells:
        i.x = (i.x+0.5) * mapResolution + msg.info.origin.position.x
        i.y = (i.y+0.5) * mapResolution + msg.info.origin.position.y
    
    #now publish the list of frontier cells
    result.header = msg.header
    result.cell_height = mapResolution
    result.cell_width = mapResolution
    goal_pub.publish(result)   
    print result

if __name__ == "__main__":
    global goal_pub
    
    rospy.init_node("Barth_Sorrells_wu_frontier_cell_finder")
    
    #publish goals
    goal_pub = map_pub = rospy.Publisher('/final_project/goals', GridCells)
    
    #subscribe to the global costmap
    #map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)
    
    rospy.sleep(rospy.Duration(1.0))
    
    print "starting frontier cell finder"
    
    rospy.spin()
    
    print "ending frontier cell finder"
