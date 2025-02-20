import os, time
import numpy as np
from copy import deepcopy
from typing import List, Dict

import rclpy

from xbot2_interface_python import pyxbot2_interface as xbi
from xbot2_interface_python.pyaffine3 import Affine3

from pyopensot.tasks.velocity import Postural, Cartesian, CollisionAvoidance as CollisionAvoidanceTask
from pyopensot.constraints.velocity import JointLimits, VelocityLimits, CollisionAvoidance as CollisionAvoidanceConstr
import pyopensot as pysot

from cartesio_planning import pycartesio_planning as pl

np.set_printoptions(precision=2, suppress=True, linewidth=1000, threshold=1000)

# init rclcpp and rclpy
pl.ros.init_rclcpp('planning_scene_test_node_cpp')
rclpy.init()
node = rclpy.create_node('planning_scene_test_node')

# load urdf, srdf
import nb_location
curr_dir = nb_location.get_nb_location()
urdf = open(curr_dir + '/../../test/resources/kyon_capsule.urdf', 'r').read()
srdf = open(curr_dir + '/../../test/resources/kyon_capsule.srdf', 'r').read()

# build model
model = xbi.ModelInterface2(urdf, srdf, 'pin')
qhome = model.getRobotState('home')

fb_lim = np.array([1, 1, 1, 3, 3, 3])
qmin, qmax = deepcopy(model.getJointLimits())
qmin[:6] = -fb_lim
qmax[:6] = fb_lim
model.setJointLimits(qmin, qmax)

# visualization
goalviz = pl.ros.RobotViz(model, 'goal', [0, 1, 0, 1])  # green
planviz = pl.ros.RobotViz(model, 'plan', [1, 1, 0, 1])  # yellow

# wrap model into a state space for planning
ss = pl.StateSpace()
ss.addRobotConfigurationSpace(model)

# define class for opensot-based constraints
class KyonGoalIk(pl.Constraint):
    
    def __init__(self, 
                 ss: pl.StateSpace, 
                 tasks: List[str], 
                 indices: List[int] = [0, 1, 2, 3, 4, 5], 
                 indices_overrides: Dict[str, List[int]] = {}, 
                 options: str = ''):
        
        super().__init__(ss, options)
        
        # define main tasks (the ones that define manifold)
        self.ee_tasks = [
            Cartesian(f'Cartesian_{ee}', model, ee, 'world') for ee in tasks
        ]
        
        # set body jacobian flag so that indices act locally
        for t in self.ee_tasks:
            t.setIsBodyJacobian(True)
        
        ee_subtasks = [t % indices_overrides.get(t.distalLink, indices) for t in self.ee_tasks]
        
        self.ee_task_aggr = sum(ee_subtasks[1:], start=ee_subtasks[0])
        
        self.csize = self.ee_task_aggr.getb().size
        
        # postural
        postural = Postural(model)
        postural.setReference(qhome)
        postural.setLambda(0.1)
        
        # collision avoidance
        self.collision_constr = CollisionAvoidanceConstr(model, max_pairs=20, skip_infeasible_pairs=False)
        # self.collision_constr.setBoundScaling(0.1)
        self.collision_constr.setDetectionThreshold(0.05)
        collision_task = CollisionAvoidanceTask(self.collision_constr)
        
        aggr = (collision_task + 0.001*postural) << self.ee_task_aggr << JointLimits(model, qmax, qmin)
        
        self.solver = pysot.iHQP([aggr])
        self.autostack = aggr
    
    def reset(self):
        for t in self.ee_tasks:
            t.reset()
            
    def solve(self, dt=1.0):
        q = model.q.copy()
        self.autostack.update()
        
        cost = np.linalg.norm(self.autostack.getb())
        
        v = self.solver.solve()
        
        alpha = 1.0
        
        while True:
            model.q = model.sum(q, alpha*v)
            model.update()
            self.autostack.update()
            cost_upd = np.linalg.norm(self.autostack.getb())
            if cost_upd < cost:
                break
            alpha *= 0.5
            if alpha < 1e-6:
                break
        
        return v*alpha
        
    def _value(self, q):
        
        model.q = q
        model.update()
        self.autostack.update()
        print(f'constraint norm is {np.linalg.norm(self.ee_task_aggr.getb())}')
        return -self.ee_task_aggr.getb()
    
    def _jacobian(self, q):
        model.q = q
        model.update()
        self.autostack.update()
        return self.ee_task_aggr.getA()
    
    def _constraintSize(self):
        return self.csize
    

# create goal ik
ee_list = [f'contact_{i+1}' for i in range(4)] + ['dagana_1_base']
indices_overrides = {'dagana_1_base': [0, 1, 2, 3, 4]}
goal_ik = KyonGoalIk(ss, ee_list, indices=[0, 1, 2], indices_overrides=indices_overrides)
goal_ik.bind(ss)

# reset from homing posture
model.q = qhome
model.update()
goal_ik.reset()

# set dagana pos ref
goal_ik.ee_tasks[-1].setReference(Affine3(pos=[0.7, 0.0, -0.2], rot=[1, 0, 0, 0]))

# sample goal from manifold

# first project qhome onto manifold
qguess = np.array(qhome)
qguess = goal_ik.project(qguess)
goalviz.publishMarkers()

dagana_pose = model.getPose('dagana_1_base')
print(dagana_pose)

# check validity
valid = ss.checkValid(qguess)[0]
print('projected state is valid:', valid)

input()

# remove collisions
while True:
    v = goal_ik.solve()
    print('dq norm is', np.linalg.norm(v))
    print('collision error', goal_ik.collision_constr.getbUpperBound())
    goalviz.publishMarkers()
    # input()