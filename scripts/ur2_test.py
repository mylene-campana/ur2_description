#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.ur2_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
import matplotlib.pyplot as plt
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')
robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
#Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
cl.obstacle.loadObstacleModel('ur2_description','cylinder_obstacle','')
r.loadObstacleModel ("ur2_description","cylinder_obstacle","cylinder_obstacle") # ??


# q = [x, y] # limits in URDF file
q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)
cl.problem.solve ()
cl.problem.optimizePath(1)


begin=time.time()
cl.problem.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
cl.problem.pathLength(0)
cl.problem.pathLength(1)

#####################################################################

## DEBUG commands
robot.setCurrentConfig(q2)
robot.collisionTest()
robot.distancesToCollision()
from numpy import *
argmin(robot.distancesToCollision()[0])
r( cl.problem.configAtDistance(0,5) )
ps.clearRoadmap ()

