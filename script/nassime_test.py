from hpp.corbaserver import * 
from hpp.corbaserver.nassime import RobotCursor

robot = RobotCursor ('robot_cursor', True)
robot.setJointBounds ("base_joint_xyz", [-4, -2, -5, -3, -1, 2])
robot.tf_root = 'base_link'


from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
v = Viewer (ps)


q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [-2.6, -4, 0.3]
v (q_init)
q_goal [0:3] = [-2.6, -4, 1.2]
v (q_goal)

v.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.clearRoadmap()

ps.selectPathPlanner("interactive")

ps.solve ()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, v)

pp (0)
pp (1)

from hpp.gepetto.viewer import GuiClient
c = GuiClient()
c.gui.createGroup("scene_hpp_/curseur2")
c.gui.addLandmark("scene_hpp_/curseur2", 1)
v.client.gui.applyConfiguration("scene_hpp_/curseur2",q_init)
#v.client.gui.refresh()
