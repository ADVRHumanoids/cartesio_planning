# main imports
import numpy as np
from copy import deepcopy
import time 

import rclpy
import rclpy.qos

from cartesio_planning import pycartesio_planning as pl
from xbot2_interface_python import pyxbot2_interface as xbi
from xbot2_interface_python.pyaffine3 import Affine3
import std_msgs.msg

np.set_printoptions(precision=2, suppress=True)

# init rospy and roscpp
pl.ros.init_rclcpp('planning_scene_test_node_cpp')
rclpy.init()
node = rclpy.create_node('planning_scene_test_node')

# load urdf, srdf
import nb_location
curr_dir = nb_location.get_nb_location()
urdf = open(curr_dir + '/../../test/resources/centauro_capsule.urdf', 'r').read()
srdf = open(curr_dir + '/../../test/resources/centauro_capsule.srdf', 'r').read()

# publish urdf to ros topic 
# use transient local qos
qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
urdf_pub = node.create_publisher(std_msgs.msg.String, '~/robot_description', qos)
urdf_msg = std_msgs.msg.String(data=urdf)
urdf_pub.publish(urdf_msg)

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')

# robotviz (i.e., marker arrays to visualize the start, goal and planned trj in rviz)
goalviz = pl.ros.RobotViz(model, 'goal', [0, 1, 0, 1])  # green
planviz = pl.ros.RobotViz(model, 'plan', [1, 1, 0, 1])  # yellow

# change joint limits to base (to reduce planning space for the base)
fb_lim = np.array([1, 1, 1, 3, 3, 3])
qmin, qmax = deepcopy(model.getJointLimits())
qmin[:6] = -fb_lim
qmax[:6] = fb_lim
model.setJointLimits(qmin, qmax)
model.q = model.getRobotState('home')
model.update()
qstart = np.array(model.q)

# wrap model into a state space for planning
ss = pl.StateSpace()
ss.addRobotConfigurationSpace(model)


## VALIDITY CHECKER ##
# create planning scene wrapper
ps = pl.ros.PlanningSceneWrapper(model)
ps.update()

# add a table and a box to the environment
ps.addBox('table', [0.8, 3.0, 0.5], Affine3(pos=[1.0, 0, -0.4]), frame_id='pelvis')
ps.addBox('box', [.2, .4, .4], Affine3(pos=[0.8, 0, 0.05]), frame_id='pelvis')

# create state validity checker from planning scene, and add it to the state space
pc = pl.ros.PlanningSceneChecker(ps, ss)
print(pc.checkValid(model.q), pc.getInvalidStateInformation())
ss.addStateValidityChecker(pc)


## PLANNING MANIFOLD
# create constact constraint (manifold) and add it to the state space
contact_map = {
    f'contact_{i+1}': [0, 1, 2] for i in range(4)
}
contact = pl.ContactConstraint(ss, model, contact_map)
contact.bind(ss)
ss.setConstraint(contact)


## GOAL SAMPLER
# create goal sampler
goal_map = contact_map.copy()

goal_map.update({
'arm1_8': [0, 1, 2],
'arm2_8': [0, 1, 2],
    })

goal = pl.ContactConstraint(ss, model, goal_map)
goal.bind(ss)

goal.setContactPose('arm1_8', Affine3(pos=[0.8, 0.35, 1.0]))
goal.setContactPose('arm2_8', Affine3(pos=[0.8, -0.35, 1.0]))

# we sample near the start pose, projected onto the goal manifold
qguess = np.array(model.q)
qguess = goal.project(qguess)

# maybe we're lucky and qguess is valid
if ss.checkValid(qguess)[0]:
    model.q = qguess
    model.update()
    goalviz.publishMarkers()
    qgoal = qguess
    print('initial guess is valid -> setting qgoal = ', qguess)
else:
    # not so lucky
    stddev = 0.3
    while True:
        qgoal = goal.sampleGaussian(qguess, stddev)
        model.q = qgoal
        model.update()
        goalviz.publishMarkers()

        if ss.checkValid(qgoal)[0]:
            goalviz.publishMarkers()
            print('found goal -> setting qgoal = ', qgoal)
            break


input('ENTER to plan')


## PLANNING
# create planner and call solve until success
planner = pl.Planner(state_space=ss)
while not planner.solve(qstart, qgoal, timeout=2, planner_type='PRMstar'):
    print('Planner failed, retrying...')

# get solution
trj = planner.getSolutionPath(simplify=True, timeout=10)
print(trj)

# interpolate
max_vel = 1.0
max_acc = 1.0
trj_dt = 0.01
time_vec, pos, vel, acc = pl.simpleTrajectoryInterpolation(ss, trj, max_vel, max_acc, trj_dt)

# visualize trj
npt = len(time_vec)
i = 0

while True:

    q = pos[:, i]
    model.q = q
    model.update()
    planviz.publishMarkers()
    i += 1
    time.sleep(trj_dt)

    if i == npt:
        i = 0
        time.sleep(1.0)

