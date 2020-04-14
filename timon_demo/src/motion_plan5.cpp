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




#include <tf/tf.h>
geometry_msgs::PoseWithCovarianceStamped button_pose {};

void printpose(const geometry_msgs::PoseWithCovarianceStamped& msg){
   
  button_pose = msg;
  //std::cout << button_pose;
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
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
  
  std::cout << move_group.getCurrentPose();
  geometry_msgs::PoseWithCovarianceStamped button_pose;
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
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 0.148886;
    target_pose.orientation.x = -0.690942;
    target_pose.orientation.y = 0.150989; 
    target_pose.orientation.z = -0.691111; 
   target_pose.position.x = 0.000603714;
   target_pose.position.y = -0.425967;
   target_pose.position.z = 0.345978;
    move_group.setPoseTarget(target_pose);

//JUNTAS
//  moveit::core::RobotStatePtr current_state0 = move_group.getCurrentState();
//  std::vector<double> joint_group_positions0;
//  current_state0->copyJointGroupPositions(joint_model_group, joint_group_positions0);
//  joint_group_positions0[0] = 0;  //positivo esquerda , negativo direita , gira a base
//  joint_group_positions0[1] = 0;  //  positivo abaixo, negativo acima, segundo link
//  joint_group_positions0[2] = 2; // positivo pra frente, negativo pra trás, terceiro link
//  joint_group_positions0[3] = 0;

//  move_group.setJointValueTarget(joint_group_positions0);

  sleep(2.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
 
  sleep(2.0);                                                    
  move_group.execute(my_plan);
  
                                                 
  sleep(2.0);
  ros::Subscriber sub = nh.subscribe("/marker_localization/pose",1, printpose);
  
  sleep(1.0);

  //std::cout << move_group.getCurrentPose();

  
   robot_state::RobotState start_state2(*move_group.getCurrentState());
   move_group.setStartState(start_state2);
//   //POSE
    target_pose.orientation.w =  0.189936;
    target_pose.orientation.x = -0.680563;
    target_pose.orientation.y = 0.192746; 
    target_pose.orientation.z = -0.680888;
    target_pose.position.x = -0.000105726;
    target_pose.position.y = -0.630134;
    target_pose.position.z = 0.239391;
   move_group.setPoseTarget(target_pose);
 
   sleep(2.0);

   move_group.plan(my_plan);
  

   sleep(2.0);  
   move_group.execute(my_plan); 



   sleep(2.0);

//     robot_state::RobotState start_state3(*move_group.getCurrentState());
//   move_group.setStartState(start_state3);
//   //POSE

//    target_pose.position.x = 0.00017605;
//    target_pose.position.y =-0.792074;
//    target_pose.position.z = 0.141905;
//   move_group.setPoseTarget(target_pose);
 
//   sleep(2.0);

//   move_group.plan(my_plan);
  

//   sleep(2.0); 
//   move_group.execute(my_plan); 
//   sleep(2.0);

//   //BACK TO UP POSITION
  
//   robot_state::RobotState start_state4(*move_group.getCurrentState());
//   move_group.setStartState(start_state4);
//  //JUNTAS 

//  moveit::core::RobotStatePtr current_state3 = move_group.getCurrentState();
//  std::vector<double> joint_group_positions3;
//  current_state3->copyJointGroupPositions(joint_model_group, joint_group_positions3);
//  joint_group_positions3[0] = 0;
//  joint_group_positions3[1] = 0;
//  joint_group_positions3[2] = 0;
//  joint_group_positions3[3] = 0;
//  move_group.setJointValueTarget(joint_group_positions3);

//  move_group.plan(my_plan);

//   sleep(2.0);

// move_group.execute(my_plan);  


//   sleep(2.0);   
//   std::cout << move_group.getCurrentPose();
  
  
  //REMOVE OBJECTS
   std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);


  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



ros::waitForShutdown(); 
  return 0;
} 