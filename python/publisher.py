import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('/planner/goal_sampler/Postural/reference', JointState, queue_size = 10)
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal_pose = JointState ()
        goal_pose.header = Header()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.name = ['VIRTUALJOINT_1', 'VIRTUALJOINT_2', 'VIRTUALJOINT_3', 'VIRTUALJOINT_4', 'VIRTUALJOINT_5', 'VIRTUALJOINT_6',
  			 'LHipLat', 'LHipSag', 'LHipYaw', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipLat', 'RHipSag',
 			 'RHipYaw', 'RKneePitch',' RAnklePitch', 'RAnkleRoll', 'WaistLat', 'WaistYaw', 'LShSag', 'LShLat',
 			 'LShYaw', 'LElbj', 'LForearmPlate', 'LWrj1', 'LWrj2', 'RShSag', 'RShLat', 'RShYaw', 'RElbj', 'RForearmPlate',
 			 'RWrj1', 'RWrj2']
        goal_pose.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3, -0.6, 0.0, 0.0, 0.0]
        goal_pose.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal_pose.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rospy.loginfo(goal_pose)
        pub.publish(goal_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
