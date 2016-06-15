from hpp.corbaserver import * 
from hpp.corbaserver.nassime import RobotL
#from hpp.corbaserver.nassime import RobotCursor
robot = RobotL('robot_L', True)
robot.setJointBounds ("base_joint_xyz", [-2.5, 12, -7, 7, -7, 7])
robot.tf_root = 'base_link'
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)


from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [0, 0, 0]
vf (q_init)
q_goal [0:7] = [10, 6, 0.11, 0.707460067, 0.706753314, 0, 0]
q_goal [0:7] = [9.7, 6, 0.11, 0.707460067, 0.706753314, 0, 0]

vf (q_goal)

vf.loadObstacleModel ("iai_maps", "env_complexe", "simple")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.clearRoadmap()

ps.selectPathPlanner("interactive")

white=[1.0,1.0,1.0,1.0]
brown=[0.85,0.75,0.15,0.5]
black=[0, 0, 0, 1]

r = vf.createViewer()
ps.addPathOptimizer ("RandomShortcut")
#print r.solveAndDisplay("rm1",1,white,0.00,1,black)

#v.displayRoadmap( "rmR1"  ,0.02,1,v.color.blue ,v.color.lightBlue ,'base_link')
r.solveAndDisplay("rmL1",2,0.02,1,r.color.green,r.color.lightGreen,'base_to_up')

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)
#vf(q_goal)
pp (0)
pp (1)


