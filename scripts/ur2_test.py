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
import numpy as np

sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')
robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
r = Viewer (ps)
pp = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
cl.obstacle.loadObstacleModel('ur2_description','cylinder_obstacle','')
r.loadObstacleModel ("ur2_description","cylinder_obstacle","cylinder_obstacle")


# q = [x, y] # limits in URDF file
q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

ps.solve ()
len(ps.getWaypoints (0))
ps.pathLength(0)
begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Optim time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(1)

begin=time.time()
ps.optimizePath (1)
end=time.time()
print "Optim2 time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(2)


# Constraint computation verification :
nSegm=4 # Number of the segment where collision has occured
beta=607022 # Collision abcisse on segment
qconstrx0 = np.array(ps.getWaypoints (0)[nSegm])*(1-beta) + beta*np.array(ps.getWaypoints (0)[nSegm+1])


#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
from numpy import *
argmin(robot.distancesToCollision()[0])
r( ps.configAtParam(0,5) )

