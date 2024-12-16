# main imports
import os
import numpy as np
from cartesio_planning import pycartesio_planning as pl
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision as coll
from copy import deepcopy
from matplotlib import pyplot as plt

np.set_printoptions(precision=2, suppress=True)

# load urdf, srdf
import nb_location
curr_dir = nb_location.get_nb_location()
urdf = open(curr_dir + '/../../test/resources/centauro_capsule.urdf', 'r').read()
srdf = open(curr_dir + '/../../test/resources/centauro_capsule.srdf', 'r').read()

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')

# change joint limits to base
fb_lim = np.array([1, 1, 1, 3, 3, 3])
qmin, qmax = deepcopy(model.getJointLimits())
qmin[:6] = -fb_lim
qmax[:6] = fb_lim
model.setJointLimits(qmin, qmax)

# wrap model into a state space for planning
ss = pl.StateSpace()
ss.addRobotConfigurationSpace(model)

# create contact constraints
contact_map = {
    f'wheel_{i+1}': [0, 1, 2] for i in range(4)
}

contact = pl.ContactConstraint(ss, model, contact_map, '')
contact.bind(ss)
ss.setConstraint(contact)

# change robot posture and use it to reset contact locations
qhome = model.getRobotState('home')
model.q = qhome
model.update()
contact.resetContactPose()

# sample a goal from constraint mainifold
qgoal = contact.sample()
print(f'constraint norm is {np.linalg.norm(contact.value(qgoal))}')

# plan
planner = pl.Planner(state_space=ss)
while not planner.solve(qhome, qgoal, timeout=2, planner_type='RRTConnect'):
    print('Planner failed, retrying...')

# get solution
trj = planner.getSolutionPath()
print(trj)

# interpolate
time, pos, vel, acc = pl.simpleTrajectoryInterpolation(ss, trj, 1., 1., 0.01)

