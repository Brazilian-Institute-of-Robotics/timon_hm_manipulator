#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("invisible_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  ROS_INFO_NAMED( "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("End effector link: %s", move_group.getEndEffectorLink().c_str());
  
    geometry_msgs::Pose target_pose;
    move_group.setGoalOrientationTolerance(0.05); //0.3
    move_group.setPlanningTime(15);
     
    target_pose.orientation.w = 0.0597191493285;//move_group.getCurrentPose().pose.orientation.w;
    target_pose.orientation.x = -0.647448126022;
    target_pose.orientation.y = -0.759645380407;
    target_pose.orientation.z = -0.0135441256963;
    target_pose.position.x = -0.153158;  
    target_pose.position.y = 0.755479;
    target_pose.position.z = 0.238906;
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    sleep(0.1);
    while (move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
    sleep(0.1);
    }
    sleep(15);
    std::cout<<move_group.getCurrentPose().pose;

    target_pose.position.z += 0.2;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1; 
    move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    sleep(5.0);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = -0.14;
    joint_group_positions[2] = 2.2;
    joint_group_positions[3] = 0;
    joint_group_positions[4] = 0.0;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);

  ros::shutdown(); 
  return 0;
} 

  