To run, put somuchstuff.launch and local_costmap_params2.yaml on the desktop of the turtlebot.  
Then launch somuchstuff.launch using roslaunch, and include a parameter for which robot it is running on.  
For example, "roslaunch somuchstuff.launch robot:=hawkeye"
After that has started, launch final_project.launch either on the turtlebot or another computer.  This launch file does not need any parameters.  
Once rviz opens, in rviz open the lab3config.rviz.
Then wait 20 minutes while the robot maybe maps some stuff.  

Astar_server.py
This node provides the A* service for the path_following node.  

FindPath.srv
This is the file that defines the service of the A*

final_project.launch
This launches the three nodes that run on my laptop (not on the turtlebot).  They are Astar_server.py, path_following.py, and frontier_finder.py.  
You can run these on the turtlebot if you want to.  

frontier_finder.py
This node finds all the unknown cells that are next to free space.  

lab3config.rviz
This has the rviz configuration that was used for the final project.  

local_costmap_params2.yaml
This file provides parameters for the local costmap.  It should be placed on the desktop of the turtlebot.  

path_following.py
This is the main node that gets maps and frontier cells, calls the A* service to find out how to get around, then drives the path.  

somuchstuff.launch
This launches the nodes that run on the turtlebot.  It provides different parameters to gmapping than the default launch file.  

