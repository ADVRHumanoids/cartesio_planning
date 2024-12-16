import actionlib
import rospy

from cartesio_planning.msg import PlanMotionAction, PlanMotionGoal, PlanMotionResult
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped

rospy.init_node('simple_plan_client')

# client
client = actionlib.SimpleActionClient('/planner_main/plan', PlanMotionAction)

# Waits until the action server has started up and started
# listening for goals.
client.wait_for_server()

# set planning scene
apply_planning_scene = rospy.ServiceProxy('/planner_main/apply_planning_scene', ApplyPlanningScene)

req = ApplyPlanningSceneRequest()
req.scene.is_diff = True 
co = CollisionObject()
co.operation = co.ADD
co.header.frame_id = 'world'
co.id = 'myshape'
co.pose.orientation.w = 1
co.pose.position.x = 0.7
co.pose.position.y = -0.3
co.pose.position.z = 1.3
shape_pose = Pose()
shape_pose.orientation.w = 1
shape = SolidPrimitive()
shape.type = shape.CYLINDER
shape.dimensions = [0, 0]
shape.dimensions[shape.CYLINDER_HEIGHT] = 1
shape.dimensions[shape.CYLINDER_RADIUS] = 0.05
co.primitives.append(shape)
co.primitive_poses.append(shape_pose)
req.scene.world.collision_objects.append(co)

apply_planning_scene(req)


# Creates a goal to send to the action server.
goal = PlanMotionGoal()
goal.type = 'joint'
goal.max_velocity = 1.0
goal.max_acceleration = 1.0
goal.trajectory_dt = 0.01
goal.planner_timeout = 20.0
goal.planner_type = 'RRTstar'

#
goal.goal_joint.name = ['J1_E']
goal.goal_joint.position = [-1]

#
goal.joint_limit_names = [f'reference@v{i}' for i in range(6)] + [f'J_wheel_{l}' for l in ('A', 'B', 'C', 'D')]
goal.joint_limit_lower = [0] * 10
goal.joint_limit_upper = [0] * 10

# Sends the goal to the action server.
client.send_goal(goal)

# Waits for the server to finish performing the action.
client.wait_for_result()

# Prints out the result of executing the action
res : PlanMotionResult = client.get_result()  # A FibonacciResult

print(res)


