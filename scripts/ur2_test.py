#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.ur2_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client

robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
r = Viewer (ps)
pp = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
r.loadObstacleModel ("ur2_description","cylinder_obstacle","cylinder_obstacle")


# q = [x, y] # limits in URDF file
#q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
#q1 = [-0.2, 1.6, -0.4, -1.35, -0.1]; q2 = [-0.2, 0.4, 0.5, -1.35, -0.1] # with 1 cylinder
#q2 = [-0.2, 0.4, 0.5, -1.2, -0.4] # with 2 other cylinders
q1 = [1.6, -0.4, -1.35, -0.1]; q2 = [0.4, 0.5, -1.35, -0.1]; # fixed base
r(q1)
robot.isConfigValid(q1); robot.isConfigValid(q2) 

cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

# config inversion ??? viewer and hpp don't read joint in same order ??
ps.selectPathPlanner ("VisibilityPrmPlanner") 
ps.selectPathValidation ("Dichotomy", 0.)
ps.saveRoadmap ('/local/mcampana/devel/hpp/data/ur2fixed-PRM.rdm')
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))

#r.client.gui.setVisibility('ur2_robot/l_sphere2',"OFF")
#r.client.gui.setVisibility('ur2_robot/l_sphere2_second_arm',"OFF")

import numpy as np

ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.2)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
timeValuesRS = cl.problem.getTimeValues ()
gainValuesRS = cl.problem.getGainValues ()
newGainValuesRS = ((1-np.array(gainValuesRS))*100).tolist()

ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
timeValuesRS2 = cl.problem.getTimeValues ()
gainValuesRS2 = cl.problem.getGainValues ()
newGainValuesRS2 = ((1-np.array(gainValuesRS2))*100).tolist()

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 50 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 0, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 0, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-.0001, tGB+0.0001, 30, 102])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.00001)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.00001)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
tmax = max(max(tGB,tGB2),max(tGB3,tGB4))
plt.axis([-.001, tmax+0.001, 30, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------
ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05)
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4)
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------

# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.5,0.5,0.5,1])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [0,0,1,1,0,0,0])
r.client.gui.refresh ()


## Video recording
import time
pp.dt = 0.02
pp.speed=0.7
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2)
r(q1)
pp(15)
#pp(ps.numberPaths()-1)
r(q2); time.sleep(1);
r.stopCapture ()


## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4

#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.distancesToCollision()[0][argmin(robot.distancesToCollision()[0])]
r( ps.configAtParam(0,5) )
robot.getJointNames ()

## Debug Optimization Tools ##############
num_log = 5225
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath, plotBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)

# Plot points
sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

# Plot trajectories
dt = 0.06
nPath = 0
plot2DBaseCurvPath(r, cl, dt, 0, "curvPath"+str(0), [1,0.3,0,1])
plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPath"+str(ps.numberPaths()-1), [0,1,0.3,1])

jointName = 'j_object_three' # correspond to 'j_object_second_arm_two'
nPath = 0
plotDofCurvPath (r, cl, robot, dt, 0, jointName, 'path_'+str(nPath)+jointName, [1,0,0,1])
nPath = ps.numberPaths ()-1
plotDofCurvPath (r, cl, robot, dt, nPath, jointName, 'path_'+str(nPath)+jointName, [0,0,1,1])

jointName = 'j_object_second_arm_two' # correspond to 'j_object_three'


#bodyName = 'l_sphere3_0'
#plotBodyCurvPath (r, cl, robot, dt, nPath, bodyName, 'path_'+str(nPath)+bodyName, [1,0,0,1])

from viewer_display_library_OPTIM import plotPointBodyCurvPath
dt = 0.06
jointName = 'j_object_second_arm_two'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0.9,0], 'pathPoint_'+jointName, [1,0,0,1])
plotPointBodyCurvPath (r, cl, robot, dt, 13, jointName, [0,0.9,0], 'pathPointGB_'+jointName, [0,1,0,1])
plotPointBodyCurvPath (r, cl, robot, dt, 14, jointName, [0,0.9,0], 'pathPointRS_'+jointName, [0,0,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 15, jointName, [0,0.9,0], 'pathPointPRS_'+jointName, [0,1,1,1])

jointName = 'j_object_three'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0.9,0], 'pathPoint2_'+jointName, [0.6,0,0,1])
plotPointBodyCurvPath (r, cl, robot, dt, 13, jointName, [0,0.9,0], 'pathPointBG2_'+jointName, [0,0.6,0,1])
plotPointBodyCurvPath (r, cl, robot, dt, 14, jointName, [0,0.9,0], 'pathPointRS2_'+jointName, [0,0,0.6,1])
plotPointBodyCurvPath (r, cl, robot, dt, 15, jointName, [0,0.9,0], 'pathPointPRS2_'+jointName, [0,0.6,0.6,1])


# test function in cl.robot
jointPosition = robot.getJointPosition ('j_object_second_arm_two')
pointInJoint = [0,0.9,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

posAtester = [0.3951179982351258, 1.1397758536996219, 0.0] # true global pos
#posAtester = [0.597073,1.10001,6.83686e-20] # false global pos
posAtester = [0.4029312802364234, 1.100679194178558, 0.0]
posAtester = [0.5000005939072879, 1.1003446463475224, 0.0]
sphereName = "machin2"
r.client.gui.addSphere (sphereName,0.04,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()


# correcting FCL contact points:
#Correct points:
(0.372233 1.12743 0) (0.400822 1.11811 0)
u1 = np.array([0.02858,-0.0093]) # normalized: array([ 0.95092153, -0.30943213])

#Fake points:
(0.561454 1.10764 0) (0.592945 1.10142 0)
u2 = np.array([0.031491,-0.0062199]) 
u2/(math.sqrt(u2[0]**2 + u2[1]**2)) # normalized: array([ 0.98104697, -0.19377009])
