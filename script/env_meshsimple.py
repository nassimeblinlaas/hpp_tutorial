from hpp.corbaserver import * 

#from hpp.corbaserver.nassime import RobotChaise
#robot = RobotChaise ('robot_chaise', True)
#from hpp.corbaserver.nassime import Robot3Angles
#robot = Robot3Angles ('robot_3Angles', True)
#from hpp.corbaserver.nassime import RobotBigMesh
#robot = RobotBigMesh ('robot_bigmesh', True)
from hpp.corbaserver.nassime import RobotMesh
robot = RobotMesh ('robot_cube_mesh', True)

robot.setJointBounds ("base_joint_xyz", [-4, 12, -4, 12, -2, 4])
#robot.setJointBounds ("base_joint_xyz", [-4, 20, -4, 12, -12, 10])
#robot.setJointBounds ("base_joint_xyz", [-1, 1, -0.5, 1.5, -0.5, 1.5])#pour les petits meshs
robot.tf_root = 'base_link'

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
#q_init [0:3] = [0, 1.5, 0.5]
q_init [0:3] = [1, -4, 1]
#q_init [0:7] = [0.50450, 0.13740, -0.21970, 0.5, 0.5, 0.5, 0.5]
v (q_init)
q_goal [0:3] = [4, 8, -4]
q_goal [0:3] = [0, 1, 3]
q_goal [0:3] = [0, 0, -2]
#q_goal [0:7] = [0, 1.1, 0, 0.5, 0.5, 0.5, 0.5]
v (q_goal)

v.loadObstacleModel ("iai_maps", "env_mesh_plat", "simple")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.clearRoadmap()

ps.selectPathPlanner("interactive")

white=[1.0,1.0,1.0,1.0]
brown=[0.85,0.75,0.15,0.5]
red=[1,0,0,1]
ps.addPathOptimizer ("RandomShortcut")
v.solveAndDisplay("rm1",1,red,0.00,1,red)
#ps.solve()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)
v(q_goal)
pp (0)
#pp (1)
