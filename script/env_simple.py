from hpp.corbaserver import * 
from hpp.corbaserver.nassime import RobotSphere

robot = RobotSphere ('robot_sphere', True)
robot.setJointBounds ("base_joint_xyz", [-2, 12, 0, 12, 0, 1])
robot.tf_root = 'base_link'

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
#q_init [0:3] = [0, 1.5, 0.5]
q_init [0:3] = [1, 2, 0]
v (q_init)
q_goal [0:3] = [9.15, 7.5, 0.5]
#v (q_goal)

v.loadObstacleModel ("iai_maps", "env_simple", "simple")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.clearRoadmap()

ps.selectPathPlanner("interactive")

#from hpp.gepetto.viewer import GuiClient
#c = GuiClient()
#c.gui.createGroup("scene_hpp_/curseur2")
#c.gui.addLandmark("scene_hpp_/curseur2", 1)
#v.client.gui.applyConfiguration("scene_hpp_/curseur2",q_init)
#v.client.gui.applyConfiguration("scene_hpp_/curseur2",q_init)
#v.client.gui.refresh()

white=[1.0,1.0,1.0,1.0]
brown=[0.85,0.75,0.15,0.5]
ps.addPathOptimizer ("RandomShortcut")
v.solveAndDisplay("rm1",50,white,0.05,1,brown)
#ps.solve()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)
pp (0)
pp (1)


