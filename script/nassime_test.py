from hpp.corbaserver import * 
from hpp.corbaserver.nassime import Robot

robot = Robot ('robot_sphere_nassime', True)

robot.setJointBounds ("base_joint_xyz", [-4, -2, -5, -3, -1, 2])

robot.tf_root = 'base_link'


from hpp_ros import ScenePublisher
r = ScenePublisher (robot)

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [-2.6, -4, 0.3]
r (q_init)
q_goal [0:3] = [-2.6, -4, 1.2]
r (q_goal)

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.clearRoadmap();
ps.solve ()

from hpp_ros import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)
