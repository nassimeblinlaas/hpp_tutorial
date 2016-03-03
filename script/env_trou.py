from hpp.corbaserver import * 
from hpp.corbaserver.nassime import Robot3Angles
robot = Robot3Angles ('robot_3angles', True)

robot.setJointBounds ("base_joint_xyz", [0, 4.375, -0, 4.375, -5, 7])
robot.tf_root = 'base_link'

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [1.5, 0.5, -3.2]
v (q_init)
q_goal [0:3] = [1.5, 1.5, 5.5]
v (q_goal)

v.loadObstacleModel ("iai_maps", "env_trou", "simple")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.clearRoadmap()

ps.selectPathPlanner("interactive")
white=[1.0,1.0,1.0,1.0]
brown=[0,0,0,1]
ps.addPathOptimizer ("RandomShortcut")
v.solveAndDisplay("rm1",50,white,0.00,1,brown)

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)
v(q_goal)
pp (0)
pp (1)


