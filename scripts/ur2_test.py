#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.ur2_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import sys
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')
robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
r = Viewer (ps)
pp = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
r.loadObstacleModel ("ur2_description","cylinder_obstacle","cylinder_obstacle")


# q = [x, y] # limits in URDF file
#q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
q1 = [-0.2, 1.6, -0.4, -1.35, -0.1];
q2 = [-0.2, 0.4, 0.5, -1.2, -0.4] # with 2 other cylinders
#q2 = [-0.2, 0.4, 0.5, -1.35, -0.1]; # with one cylinder
r(q1)
#robot.isConfigValid(q1); robot.isConfigValid(q2) 

cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

# config inversion ??? viewer and hpp don't read joint in same order ??
#ps.selectPathPlanner ("VisibilityPrmPlanner") 
ps.solve ()
ps.pathLength(0)

ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)


for i in range(0,ps.numberPaths()):
    print ps.pathLength(i)

len(ps.getWaypoints (0))

pp.dt = 0.02

r.startCapture ("capture","png")
pp(ps.numberPaths()-1)
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4
#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.distancesToCollision()[0][argmin(robot.distancesToCollision()[0])]
r( ps.configAtParam(0,5) )

