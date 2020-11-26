#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion

from cartesio_planning.srv import CartesioPlanner
from cartesio_planning.msg import SetContactFrames

from std_srvs.srv import Empty

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

import numpy as np
import yaml

class planner_client(object):
    def __init__(self):
        self.__manifold_str = rospy.get_param("planner/problem_description_constraint")
        self.__manifold_dict = yaml.safe_load(self.__manifold_str)

        self.__start_pub = rospy.Publisher('planner/start/joint_states', JointState, queue_size=10, latch=True)
        self.__goal_pub = rospy.Publisher('planner/goal/joint_states', JointState, queue_size=10, latch=True)
        self.__contact_pub = rospy.Publisher('planner/contacts', SetContactFrames, queue_size=10, latch=True)

        rospy.wait_for_service('/planner/reset_manifold')
        self.__reset_manifold = rospy.ServiceProxy('planner/reset_manifold', Empty)

    def publishStartAndGoal(self, joint_names, start, goal):
        start_msg, goal_msg = self.__createStartAndGoalMsgs(joint_names, start, goal)
        self.__start_pub.publish(start_msg)
        self.__goal_pub.publish(goal_msg)

    def publishContacts(self, contact_dict):
        """
        configs_contacts.append({"r_ball_tip": [0, 0, 0, 1] , "l_sole": [0, 0, 0, 1] , "r_sole": [0, 0, 0, 1]})
        """
        contact_msg = self.__createContactFrameMsg(contact_dict)
        self.__contact_pub.publish(contact_msg)

    def updateManifold(self, new_manifold):
        """
        To update used manifold with new manifold list ["link1", "link2", ...]
        """
        to_param = False
        if sorted(self.__manifold_dict["stack"][0]) != sorted(new_manifold):
            self.__manifold_dict["stack"][0] = new_manifold
            to_param = True

        if to_param:
            rospy.set_param("planner/problem_description_constraint", yaml.dump(self.__manifold_dict))
            rospy.sleep(1.)
            self.__reset_manifold()

    def solve(self, PLAN_MAX_ATTEMPTS, planner_type, plan_time, interpolation_time, goal_threshold):
        rospy.wait_for_service('planner/compute_plan')
        PLAN_ATTEMPTS = 0
        plan_success = False

        while PLAN_ATTEMPTS < PLAN_MAX_ATTEMPTS:
            try:

                plan = rospy.ServiceProxy('planner/compute_plan', CartesioPlanner)
                response = plan(planner_type=planner_type, time=(plan_time * (PLAN_ATTEMPTS + 1)), interpolation_time=interpolation_time,
                                goal_threshold=goal_threshold)
                if response.status.val == 6:  # EXACT_SOLUTION
                    print("EXACT_SOLUTION FOUND")
                    return True
                elif response.status.val == 5:  # APPROXIMATE SOLUTION
                    print("APPROXIMATE_SOLUTION FOUND")
                    PLAN_ATTEMPTS += 1
                else:
                    rospy.logerr("PLANNER RETURNED ERROR: %i", response.status.val)
                    PLAN_ATTEMPTS += 1

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False

        rospy.logerr("PLANNER CAN NOT FIND A SOLUTION, EXITING")
        return False

    ##PRIVATE
    def __createStartAndGoalMsgs(self, joint_names, start_config, goal_config):
        """
        To create start and goal msgs from start and goal configs lists
        """
        start_pose = JointState()
        goal_pose = JointState()

        for i in range(len(joint_names)):
            start_pose.name.append(joint_names[i])
            start_pose.position.append(start_config[i])
            goal_pose.name.append(joint_names[i])
            goal_pose.position.append(goal_config[i])

        return start_pose, goal_pose

    def __createContactFrameMsg(self, contact_dict):
        """
        To create message for static stability check
        """
        contact_msg = SetContactFrames()
        contact_msg.action = contact_msg.SET
        for key in contact_dict:
            contact_msg.frames_in_contact.append(key)
            quat = Quaternion()
            quat.x = contact_dict[key][0]
            quat.y = contact_dict[key][1]
            quat.z = contact_dict[key][2]
            quat.w = contact_dict[key][3]
            contact_msg.rotations.append(quat)
        contact_msg.optimize_torque = False
        contact_msg.friction_coefficient = 0.5*np.sqrt(2.)
        return contact_msg



if __name__ == '__main__':
    urdf = rospy.get_param("robot_description")
    srdf = rospy.get_param("robot_description_semantic")

    opt = co.ConfigOptions()
    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('model_type', 'RBDL')

    model = xbot.ModelInterface(opt)

    joint_names = model.getEnabledJointNames()

    configs = []
    configs.append([ -0.0519934, -0.00367742,    0.622926,    3.11831,     1.17912,      3.2064,    0.448539,    -1.50196,   0.489673,           0,   -0.452481,  -0.0825496,    -1.13953,    -1.26619,    -1.24827,           0,   -0.507944,    0.261799,   -0.150113,   -0.204497,      -3.352,     1.04099,    -1.75615,     -1.1217,    0.994067,       1.478,    0.162285,     -3.359,    -1.13074,     -1.1096,   -0.189419,    -1.9351,    -1.19221,    -2.22644])
    configs.append([-0.13821,  -0.203093,    0.657911,      3.0598,     1.15335,     3.15055,     1.42741,    -1.26646 ,     1.5708,           0,   -0.438969,   -0.261799,   -0.887553,    -1.66163,   -0.855183, 8.05454e-18,    -0.35891, -0.00299101,   0.0568406,   0.0849584,    -3.21009,     1.24629,   -0.924189,   -0.298677,    0.756945,      0.5094,   -0.669409,    -2.66191,   -0.294821,   -0.396937,       -1.02,       -2.55,    -1.36005,       -2.55])
    configs.append([-0.111, -0.171,  0.685,  3.041,  1.462,  3.199,  0.828, -1.182,  0.906,  0.,    -0.361, -0.262, -1.169, -1.608, -1.16,   0.541, -0.634,  0.008,  0.246,  0.596, -1.787,  1.625, -0.059, -1.162,  0.08,   0.033, -1.797, -2.71,  -0.53,  -0.74,  -0.541, -1.673, -1.083, -2.55 ])

    configs_contacts = []
    configs_contacts.append({"l_ball_tip": [0, 0, 0, 1], "r_ball_tip": [0, 0, 0, 1], "l_sole": [0, 0, 0, 1], "r_sole": [0, 0, 0, 1]})
    configs_contacts.append({"r_ball_tip": [0, 0, 0, 1] , "l_sole": [0, 0, 0, 1] , "r_sole": [0, 0, 0, 1]})

    quaternion_contacts = []
    quaternion_contacts.append([])


    rospy.init_node('stances_publisher', anonymous=True)


    planner_client = planner_client()

    while len(configs) > 1:
        start_config = configs.pop(0)
        goal_config = configs[0]
        config_contacts = configs_contacts.pop(0)

        planner_client.updateManifold(config_contacts.keys())

        for i in range(10):
            planner_client.publishStartAndGoal(joint_names, start_config, goal_config)
            planner_client.publishContacts(config_contacts)
            rospy.sleep(0.1)

        if planner_client.solve(PLAN_MAX_ATTEMPTS=5, planner_type="RRTConnect", plan_time=30, interpolation_time=0.01, goal_threshold=1):
            raw_input("Press a button for nex configuration")
        else:
            print("ERROR! PLANNER CAN NOT FIND A SOLUTION!")
            break

