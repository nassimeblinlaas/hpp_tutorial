from hpp.corbaserver import * 
from hpp.corbaserver.nassime import RobotCursor

robot = RobotCursor ('robot_cursor', True)
robot.setJointBounds ("base_joint_xyz", [0, 9.5, 0, 12, 0, 1])
robot.tf_root = 'base_link'

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [0, 1.5, 0.5]
v (q_init)
q_goal [0:3] = [9.15, 7.5, 0.5]
v (q_goal)

v.client.gui.applyConfiguration("scene_hpp_/robot_cursor",q_init)

v.loadObstacleModel ("iai_maps", "labyrinth2", "labyrinth")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.clearRoadmap()
ps.selectPathPlanner("interactive")

white=[1.0,1.0,1.0,1.0]
brown=[0,0,0,1]
ps.addPathOptimizer ("RandomShortcut")
v.solveAndDisplay("rm1",50,white,0.000,1,brown)
#ps.solve()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)

pp (0)
pp (1)


#from hpp.gepetto.viewer import GuiClient
#c = GuiClient()
#c.gui.createGroup("scene_hpp_/curseur2")
#c.gui.addLandmark("scene_hpp_/curseur2", 1)
#v.client.gui.applyConfiguration("scene_hpp_/curseur2",q_init)
#v.client.gui.applyConfiguration("scene_hpp_/curseur2",q_init)
#v.client.gui.refresh()
