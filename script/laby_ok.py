from hpp.corbaserver.nassime import RobotCursor


robot = RobotCursor ('robot_cursor', True)
robot.setJointBounds ("base_joint_xyz", [-4, 10, -5, 7.5, 0, 1])
robot.tf_root = 'base_link'
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
r = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [-3.2, -4, 2]
r (q_init)
q_goal [0:3] = [-3.2, -4, 0]
r (q_goal)
q_init [0:3] = [0, 1.5, 0.5]
r (q_init)
q_goal [0:3] = [9.15, 7.5, 0.5]
r (q_goal)
#r.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
r.loadObstacleModel ("iai_maps", "labyrinth2", "labyrinth")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.addPathOptimizer ("RandomShortcut")
ps.selectPathPlanner("interactive")

#changer le solve
ps.solve ()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)
