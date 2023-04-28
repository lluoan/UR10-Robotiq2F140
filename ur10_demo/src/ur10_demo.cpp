#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();  
  
  // Construct a MoveGroup object 
  moveit::planning_interface::MoveGroupInterface group("manipulator");    
  
  // Get the name of the end-effector link 
  // std::string end_effector_link = group.getEndEffectorLink();
 
  // Set the reference frame for pose targets
  group.setPoseReferenceFrame("base_link");
 
  // Pose target for the robot 
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.8;   
  target_pose1.position.x = 0.5;   
  target_pose1.position.y = 0.0;  
  target_pose1.position.z = 1.0;   

  // Set the start state for the group
  group.setPoseTarget(target_pose1);


  // Plan and execute the trajectory 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  // ROS_INFO_NAMED("state","Start state : %s", group.getCurrentPose().ToString().c_str()); 
  
  // Execute the planned trajectory
  group.move();
  // if (success){
  //   group.execute(my_plan);
  // }
  // else {
  // ROS_INFO("Planing failed!");
  // }
  // // group.execute(my_plan);
  // ROS_INFO("Done");
  
  
  ros::shutdown();
  return 0;
}
