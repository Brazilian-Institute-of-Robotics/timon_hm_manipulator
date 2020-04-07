#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>



#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "timon_arm";
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

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  std::cout << move_group.getCurrentPose();
  // sleep(15.0);
//ADDING FLOOR
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();


  // The id of the object is used to identify it.
  collision_object.id = "floor";


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 0.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.11;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);



  robot_state::RobotState start_state(*move_group.getCurrentState());
  move_group.setStartState(start_state);
 //POSE
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 0.383166;
    target_pose1.orientation.x = 0.593944;
    target_pose1.orientation.y = -0.595274; 
    target_pose1.orientation.z = 0.382181; 
   target_pose1.position.x = -0.000291728;
   target_pose1.position.y = -0.425967;
   target_pose1.position.z = 0.345978;
    move_group.setPoseTarget(target_pose1);

//JUNTAS
//  moveit::core::RobotStatePtr current_state0 = move_group.getCurrentState();
//  std::vector<double> joint_group_positions0;
//  current_state0->copyJointGroupPositions(joint_model_group, joint_group_positions0);
//  joint_group_positions0[0] = 0;  //positivo esquerda , negativo direita , gira a base
//  joint_group_positions0[1] = 0;  //  positivo abaixo, negativo acima, segundo link
//  joint_group_positions0[2] = 2; // positivo pra frente, negativo pra trás, terceiro link
//  joint_group_positions0[3] = 0;

//  move_group.setJointValueTarget(joint_group_positions0);
  // visual_tools.prompt("Press 'next'");
  sleep(2.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//  visual_tools.publishAxisLabeled(target_pose1, "pose1");
//  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
  // visual_tools.prompt("Press 'next'"); 
  sleep(2.0);                                                    
  move_group.execute(my_plan);
  
  // visual_tools.prompt("Press 'next'");                                                     
  sleep(4.0);
  std::cout << move_group.getCurrentState();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  robot_state::RobotState start_state2(*move_group.getCurrentState());
  move_group.setStartState(start_state2);
  //POSE
  geometry_msgs::Pose target_pose2;
   target_pose2.orientation.w =  0.383166;
   target_pose2.orientation.x = 0.593944;
   target_pose2.orientation.y = -0.595274; 
   target_pose2.orientation.z = 0.382181;
   target_pose2.position.x = -0.000319611;
   target_pose2.position.y = -0.71035;
   target_pose2.position.z = 0.2208;
  move_group.setPoseTarget(target_pose2);
 
  sleep(2.0);
//JUNTAS
//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
// //  joint_group_positions[0] = 0;
//   joint_group_positions[1] = -0.7;
//   joint_group_positions[2] = 1.3;
// //  joint_group_positions[3] = 0;
//  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  sleep(2.0);  
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group.execute(my_plan); 


//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  sleep(2.0);

  robot_state::RobotState start_state3(*move_group.getCurrentState());
  move_group.setStartState(start_state3);

//   //POSE
//   //  geometry_msgs::Pose target_pose3;
//   //  target_pose3.orientation.w = 0;
//   //  target_pose3.position.x = 0.0;
//   //  target_pose3.position.y = 0.1;
//   //  target_pose3.position.z = 0.99;
//   //  move_group.setPoseTarget(target_pose3);

// //JUNTAS 

 moveit::core::RobotStatePtr current_state3 = move_group.getCurrentState();
 std::vector<double> joint_group_positions3;
 current_state3->copyJointGroupPositions(joint_model_group, joint_group_positions3);
 joint_group_positions3[0] = 0;
 joint_group_positions3[1] = 0;
 joint_group_positions3[2] = 0;
 joint_group_positions3[3] = 0;
 move_group.setJointValueTarget(joint_group_positions3);

 success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  sleep(2.0);
//ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
move_group.execute(my_plan);
  
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  
  // REMOVE OBJECTS
  // visual_tools.prompt("Press 'next'");  
  sleep(2.0);   
  std::cout << move_group.getCurrentPose();
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
   std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);


  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



ros::shutdown();
  return 0;
}