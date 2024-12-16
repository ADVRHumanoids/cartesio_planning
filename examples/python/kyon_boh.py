# main imports
import numpy as np
from copy import deepcopy
import rospy
import rospkg
import os

from cartesio_planning import pycartesio_planning as pl
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface.pyaffine3 import Affine3
from xbot2_interface import pyxbot2_collision as coll

from cartesian_interface.pyci_all import *
from cartesio_collision_support.pycollision_support import CollisionTask

np.set_printoptions(precision=2, suppress=True)

# init rospy and roscpp
pl.ros.init_ros('planning_scene_test_node_cpp')
rospy.init_node('planning_scene_test_node')

# load urdf, srdf
rospack = rospkg.RosPack()
urdf = open(rospack.get_path('kyon_urdf') + '/urdf/kyon_capsule.urdf', 'r').read()
srdf = open(rospack.get_path('kyon_srdf') + '/srdf/kyon_capsule.srdf', 'r').read()
rospy.set_param('planning_scene_test_node/robot_description', urdf)

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')
# model = _model.generateReducedModel(_model.q0, ['dagana_1_clamp_joint', 'dagana_2_clamp_joint'])
# print(model.getJointNames())

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

z_contact = model.getPose('contact_1').translation[2]
model.setJointPositionMinimal({
    'shoulder_pitch_1': 0,
    'shoulder_pitch_2': 0,
    'reference@v2': -z_contact
})
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
ps.addBox('ground', [2, 2, 0.05], Affine3(pos=[0.0, 0, -0.03]))
# ps.addBox('table', [0.8, 3.0, 0.5], Affine3(pos=[1.0, 0, 0.25]))
# ps.addBox('box', [.2, .4, .4], Affine3(pos=[0.8, 0, 0.55]))

# create state validity checker from planning scene, and add it to the state space
pc = pl.ros.PlanningSceneChecker(ps, ss)
if not pc.checkValid(model.q):
    print('initial state is invalid')
    print(pc.getInvalidStateInformation())
    exit()

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
'dagana_1_base': [0, 1, 2, 3, 4],
# 'dagana_2_base': [0, 1, 2],
    })

goal = pl.ContactConstraint(ss, model, goal_map)
goal.setRefineTarget(qstart)
goal.bind(ss)

goal.setContactPose('dagana_1_base', Affine3(pos=[0.7, 0.0, 0.2], rot=[1, 0, 0, 0]))
# goal.setContactPose('dagana_2_base', Affine3(pos=[0.8, -0.35, 0.65]))

# ik pb to remove collisions
ik_pb = open(os.path.dirname(os.path.abspath(__file__)) + '/kyon_goal_sampler_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance(solver='OpenSot', problem=ik_pb, model=model, dt=1.0)
collision_task = ci.getTask('collision_avoidance')
collision_model = collision_task.getCollisionModel()

collision_model.addCollisionShape('ground', 'world', coll.Shape.Box([2, 2, 0.05]), Affine3(pos=[0, 0, -0.025]), [])
collision_model.setLinksVsEnvironment({l for l in collision_model.getLinksVsEnvironment() if 'knee_pitch' not in l})
collision_task.collisionModelUpdated()


# we sample near the start pose, projected onto the goal manifold
qguess = np.array(qstart)
qguess = goal.project(qguess)
model.q = qguess
model.update()
goalviz.publishMarkers()

input()

# we try to remove collisions
ci.reset(0)

while True:
    if not ci.update(0.0, 1.0):
        raise RuntimeError('unable to solve ik')
    qupd = model.sum(model.q, model.v)
    model.q = qupd
    model.update()
    goalviz.publishMarkers()
    if ss.checkValid(qupd)[0] and np.linalg.norm(goal.value(qupd)) < 1e-4:
        print('VALID')
        qguess = goal.refine(qupd)
        break


# maybe we're lucky and qguess is valid
if ss.checkValid(qguess)[0]:
    model.q = qguess
    model.update()
    goalviz.publishMarkers()
    qgoal = qguess
    print('initial guess is valid -> setting qgoal = ', qguess)
else:
    # not so lucky
    stddev = 1.0
    while True:
        qgoal = goal.sampleGaussian(qguess, stddev)
        model.q = qgoal
        model.update()
        goalviz.publishMarkers()

        if ss.checkValid(qgoal)[0]:
            goalviz.publishMarkers()
            input('REFINE')
            qgoal = goal.refine(qgoal)
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
print('interpolating path...')
time_vec, pos, vel, acc = pl.simpleTrajectoryInterpolation(ss, trj, max_vel, max_acc, trj_dt)
print('..done')

# visualize trj
npt = len(time_vec)
i = 0
rate = rospy.Rate(1./trj_dt)

print(f'npoints = {npt}')

while True:

    q = pos[:, i]
    print(q)
    model.q = q
    model.update()
    planviz.publishMarkers()
    i += 1
    rate.sleep()

    if i == npt:
        i = 0
        rospy.sleep(rospy.Duration(1.0))

