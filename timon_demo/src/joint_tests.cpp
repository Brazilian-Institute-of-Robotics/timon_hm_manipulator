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
#include <tf/transform_listener.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_tests");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
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
  
  
  

  robot_state::RobotState start_state(*move_group.getCurrentState());
  move_group.setStartState(start_state);

  //JUNTAS
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // //joint_group_positions[0] = 0;  //positivo esquerda , negativo direita , gira a base
  // //joint_group_positions[1] = 0;  //  positivo abaixo, negativo acima, segundo link
  // //joint_group_positions[2] = 2; // positivo pra frente, negativo pra trás, terceiro link
  // //joint_group_positions[3] = 0;
  // joint_group_positions[4] = 1.1;

  // move_group.setJointValueTarget(joint_group_positions);

  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 0.218820; // 0.474989;
  target_pose.orientation.x = 0.672372; // -0.522257;
  target_pose.orientation.y = 0.671502;  // 0.47662;
  target_pose.orientation.z = -0.221623;  // -0.523895
  target_pose.position.x = -0.013636; //0.000603714
  target_pose.position.y = 0.422737; //-0.335967
  target_pose.position.z = 0.378490; //0.305978
  move_group.setPoseTarget(target_pose);

  // sleep(2.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
 
  // sleep(2.0);                                                    
  move_group.execute(my_plan);


  std::cout << move_group.getCurrentPose();

  robot_state::RobotState start_state1(*move_group.getCurrentState());
  move_group.setStartState(start_state1);
  sleep(1.0);
  target_pose.position.z = 0.45;
  move_group.setPoseTarget(target_pose);
  move_group.plan(my_plan);
  move_group.execute(my_plan);

  tf::TransformListener listener;


  //GETTING BUTTON POSE
  tf::StampedTransform transform;
  try{
  // ros::Time now = ros::Time::now();
  listener.waitForTransform("/fake_botao", "/invisible_link", 
                           ros::Time(0), ros::Duration(2.0));
  listener.lookupTransform("/fake_botao", "/invisible_link",
                           ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
  ROS_ERROR("%s",ex.what());
  }
  if (transform.getRotation().z() >= 0.9) {
      robot_state::RobotState start_state2(*move_group.getCurrentState());
      move_group.setStartState(start_state2);
      
      //POSE
      target_pose.orientation.w = 0.011888; // 0.474989;
      target_pose.orientation.x = 0.707720; // -0.522257;
      target_pose.orientation.y = 0.706234;  // 0.47662;
      target_pose.orientation.z = -0.014941;  // -0.523895
      target_pose.position.x = -transform.getOrigin().x();
      target_pose.position.y = transform.getOrigin().y();
      target_pose.position.z = -transform.getOrigin().z();
      move_group.setGoalPositionTolerance(0.01);
      move_group.setGoalOrientationTolerance(0.01);
      move_group.setPlanningTime(10);
      move_group.setPoseTarget(target_pose);

      move_group.plan(my_plan);
      move_group.execute(my_plan);

      //PRESS BUTTON

      robot_state::RobotState start_state3(*move_group.getCurrentState());
      move_group.setStartState(start_state3);

      //  target_pose.position.y +=0.040; 
       target_pose.position.z -=0.030;       
       move_group.setPoseTarget(target_pose);
       move_group.plan(my_plan);  
       move_group.execute(my_plan); 

      robot_state::RobotState start_state4(*move_group.getCurrentState());
      move_group.setStartState(start_state4);

      //  target_pose.position.y +=0.040; 
       target_pose.position.z +=0.030;       
       move_group.setPoseTarget(target_pose);
       move_group.plan(my_plan);  
       move_group.execute(my_plan); 
      
  }
  else
  {
    robot_state::RobotState start_state4(*move_group.getCurrentState());
   move_group.setStartState(start_state4);
//   //POSE
    target_pose.orientation.w =  0.505412;
    target_pose.orientation.x = -0.496170;
    target_pose.orientation.y = -0.493397; 
    target_pose.orientation.z = -0.504908;
    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().z();
    target_pose.position.z = -transform.getOrigin().y();
    // std::cout<<"Pose alvo y: " << target_pose.position.y << "  " << "Pose alvo z: " << target_pose.position.z;

   move_group.setGoalPositionTolerance(0.01);
   move_group.setGoalOrientationTolerance(0.01);
   move_group.setPlanningTime(10);
   move_group.setPoseTarget(target_pose);
 

   move_group.plan(my_plan);  
 
   move_group.execute(my_plan); 



    //PRESS BUTTON
    robot_state::RobotState start_state5(*move_group.getCurrentState());
    move_group.setStartState(start_state5);

    target_pose.position.y +=0.031;
    move_group.setPoseTarget(target_pose);

    move_group.plan(my_plan);  

    move_group.execute(my_plan); 




    robot_state::RobotState start_state6(*move_group.getCurrentState());
    move_group.setStartState(start_state6);

    target_pose.position.y -=0.031;
    move_group.setPoseTarget(target_pose);
    
    move_group.plan(my_plan); 
    move_group.execute(my_plan);  
  }
  




  std::cout << move_group.getCurrentPose();
  ros::waitForShutdown(); 
  return 0;
} 

