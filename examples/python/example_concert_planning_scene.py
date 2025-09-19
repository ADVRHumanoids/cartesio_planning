# main imports
import rclpy
import os, time
import numpy as np
import xbot2_interface
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision as coll
from xbot2_interface.pyaffine3 import Affine3
from cartesio_planning import pycartesio_planning as pl
from copy import deepcopy

np.set_printoptions(precision=2, suppress=True)

# main options
planner_type = 'PRMstar'
planner_timeout = 2.0  # seconds
use_moveit_collision = True  # use MoveIt! collision checking

# init rclcpp
pl.ros.init_rclcpp('planning_scene_test_node_cpp')
rclpy.init()
node = rclpy.create_node('planning_scene_test_node_py')

# load urdf, srdf
import nb_location
curr_dir = nb_location.get_nb_location()
urdf = open(curr_dir + '/../../test/resources/modularbot.urdf', 'r').read()
srdf = open(curr_dir + '/../../test/resources/modularbot.srdf', 'r').read()

# publish urdf in transient topic
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
urdf_pub = node.create_publisher(String, '/robot_description', qos_profile)
urdf_msg = String()
urdf_msg.data = urdf
urdf_pub.publish(urdf_msg)

# srdf too
srdf_pub = node.create_publisher(String, '/robot_description_semantic', qos_profile)
srdf_msg = String()
srdf_msg.data = srdf
srdf_pub.publish(srdf_msg)

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')

# print joints
print('joints = ', model.getJointNames())

# lock base and wheels
fb_lim = np.array([1, 1, 1, 0, 0, 10]) * 0
qmin, qmax = deepcopy(model.getJointLimits())
qmin[:6] = -fb_lim
qmax[:6] = fb_lim
qmin[6:14] = 0
qmax[6:14] = 0
model.setJointLimits(qmin, qmax)

# wrap model into a state space for planning
ss = pl.StateSpace()
ss.addRobotConfigurationSpace(model)

# define start and goal in joint space
qhome = model.getRobotState('home').copy()
qstart = qhome.copy()
qstart[model.getJointInfo('J1_E').iq] = -2.0
qgoal = qhome.copy()
qgoal[model.getJointInfo('J1_E').iq] = 2.0

# publish start and goal
goalviz = pl.ros.RobotViz(model, 'goal', [0, 1, 0, 1])  # green
planviz = pl.ros.RobotViz(model, 'plan', [1, 1, 0, 1])  # yellow

model.q = qstart
model.update()
planviz.publishMarkers()

model.q = qgoal
model.update()
goalviz.publishMarkers()

# create a collision model
ps = pl.ros.PlanningSceneWrapper(model)

box = coll.Box()
box.size = [0.5, 0.5, 0.5]
w_T_box = Affine3(pos=[1, 0, 1.25])

ps.addBox(id='box1', size=box.size, T=w_T_box)
ps.update()

self_collisions = ps.checkSelfCollisions()
print('Self collisions:', self_collisions)
colliding_links = ps.getCollidingLinks()
print('Colliding links:', colliding_links)

if use_moveit_collision:
    # add it to state space 
    ps_checker = pl.ros.PlanningSceneChecker(ps, ss)
    ss.addStateValidityChecker(ps_checker)
else:
    # create a collision model
    collision_model_opt = coll.CollisionModel.Options()
    collision_model_opt.assume_convex_meshes = True
    collision_model = coll.CollisionModel(model, collision_model_opt)
    collision_model.addCollisionShape(
        name='box1',
        link='world',
        shape=box,
        link_T_shape=w_T_box,
    )
    coll_checker = pl.CollisionValidityChecker(ss, collision_model)
    ss.addStateValidityChecker(coll_checker)

# plan
planner = pl.Planner(state_space=ss)
ok = planner.solve(qstart, qgoal, timeout=planner_timeout, planner_type=planner_type)
assert ok, 'planner failed'

# get solution
trj = planner.getSolutionPath()

# interpolate
tic = time.time()
time_vec, pos, vel, acc = pl.simpleTrajectoryInterpolation(ss, trj, 1., 1., 0.01)
print(f'Interpolation took {time.time()-tic} seconds')

# play
ntrj = len(time_vec)
i = 0
while rclpy.ok():
    model.q = pos[:, i]
    model.update()
    planviz.publishMarkers()
    dt = time_vec[i+1] - time_vec[i] if i < ntrj - 1 else 1.0
    i = (i + 1) % ntrj
    time.sleep(dt)

rclpy.spin(node)