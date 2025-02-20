# main imports
import os, time
import numpy as np
from xbot2_interface_python import pyxbot2_interface as xbi
from xbot2_interface_python import pyxbot2_collision as coll
from cartesio_planning import pycartesio_planning as pl
from copy import deepcopy

np.set_printoptions(precision=2, suppress=True)

# load urdf, srdf
import nb_location
curr_dir = nb_location.get_nb_location()
urdf = open(curr_dir + '/../../test/resources/centauro_capsule.urdf', 'r').read()
srdf = open(curr_dir + '/../../test/resources/centauro_capsule.srdf', 'r').read()

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')

fb_lim = np.array([1, 1, 1, 3, 3, 3])
qmin, qmax = deepcopy(model.getJointLimits())
qmin[:6] = -fb_lim
qmax[:6] = fb_lim
model.setJointLimits(qmin, qmax)

# wrap model into a state space for planning
ss = pl.StateSpace()
ss.addRobotConfigurationSpace(model)

# use state space to generate a random config
qrand = ss.random()
print(f'qrand = {qrand}')

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

assert model.checkJointLimits(qgoal), 'goal violates joint limits'

# plan
planner = pl.Planner(state_space=ss)
ok = planner.solve(qhome, qgoal, timeout=2, planner_type='RRTConnect')
assert ok, 'planner failed'

# get solution
trj = planner.getSolutionPath()

# interpolate
tic = time.time()
time_vec, pos, vel, acc = pl.simpleTrajectoryInterpolation(ss, trj, 1., 1., 0.01)
print(f'Interpolation took {time.time()-tic} seconds')

# evaluate constraint on interpolated trajectory
constr = [contact.value(q) for q in pos.T]

# plot
import matplotlib
import matplotlib.pyplot as plt

plt.figure()
plt.subplot(2, 2, 1)
plt.title('Trajectory')
axes = plt.plot(time_vec, pos.T)
plt.grid()

plt.subplot(2, 2, 2)
plt.title('Velocities')
axes = plt.plot(time_vec, vel.T)
plt.grid()

plt.subplot(2, 2, 3)
plt.title('Accelerations')
axes = plt.plot(time_vec, acc.T)
plt.grid()

plt.subplot(2, 2, 4)
plt.title('Constraint violation')
axes = plt.plot(time_vec, constr)
plt.grid()

plt.tight_layout()

plt.show()