import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# Get the name of the end-effector link
end_effector_link = group.get_end_effector_link()

# Set the reference frame for pose targets
group.set_pose_reference_frame("base_link")

# Pose target for the arm
target_pose = Pose()
target_pose.orientation.w = 1.0  
target_pose.position.x = 0.5  
target_pose.position.y = 0.0 
target_pose.position.z = 1.0  

# Set the start state for the arm  
group.set_start_state(target_pose)

# Plan and execute a trajectory to the target pose 
plan1 = group.go(wait=True)  

# Calling `stop()` ensures that there is no residual movement
group.stop()

# Close the connection to the move_group
moveit_commander.roscpp_shutdown()