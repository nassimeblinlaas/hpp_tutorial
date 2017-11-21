
from hpp.corbaserver import * 

#from hpp.corbaserver.nassime import RobotSiege
#robot = RobotSiege ('robot_siege', True)
from hpp.corbaserver.nassime import RobotMesh3angles
robot = RobotMesh3angles ('robot_mesh_3angles', True)
#from hpp.corbaserver.nassime import Robot3Angles
#robot = Robot3Angles ('robot_L', True)
#from hpp.corbaserver.nassime import RobotStrange
#robot = RobotStrange ('robot_strange', True)

robot.setJointBounds ("base_joint_xyz", [-0.25, 0.20, -0.20, 0.25, -1, 0.7])#pour les petits meshs
robot.tf_root = 'base_link'

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:7] = [0,0,-0.5,1,0,0,0]
# pour l'encombre
q_init [0:7] = [0,0,-0.7,1,0,0,0]
q_init [0:7] = [0.025,0.05,-0.72,1,0,0,0]
v (q_init)
q_goal [0:7] = [0.05, 0, 0.3, 1, 0, 0, 0]
v (q_goal)

v.loadObstacleModel ("iai_maps", "env_mesh_roman", "simple")
#v.loadObstacleModel ("iai_maps", "env_mesh_roman_e", "simple")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.clearRoadmap()

ps.selectPathPlanner("interactive")

white=[1.0,1.0,1.0,1.0]
brown=[0.85,0.75,0.15,0.5]
black=[0, 0, 0, 1]     
ps.addPathOptimizer ("RandomShortcut")
v.solveAndDisplay("rm1",1,white,0.00,1,black)
#ps.solve()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)
v(q_goal)
pp (0)
#pp (1)