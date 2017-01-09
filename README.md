# RoboManip_ClassProj
Cornell Robotic Manipulation Course: Class projects

Motion Planner
The motion planner's function is to help the Baxter robot play in a "soccer match". The opposing team's "players" are blocks,
and Baxter wants to score by throwing a ball that avoids these blocks. The purpose of this code is to help Baxter decide where
to throw the ball and avoid all obstacles. It does this by examining multiple linear planar trajectories and selects the 
trajectory that goes in the goal with the highest distance from obstacles.

RRT Builder
Rapidly exploring random tree (RRT) is a method to explore a robot's space and move from the start position to a goal position.
In this case, the positions are actually robot arm configurations. The catch is that there are obstacles in between the 
2 configurations that must be avoided. To get around these obstacles an RRT (in this case bi-directional) is built that 
expands outwards from both the start and goal position in small steps. Eventually, it connects the two trees and then follows
this path from start to end configurations. Another nice addition to this is smoothing the path once it is found to prevent jerky
robot arm movement.

Note: Both these codes are a little rough. The projects in this semester are my first foray into Python, so please excuse 
some of the structure and syntax. Both codes performed well in the class.
