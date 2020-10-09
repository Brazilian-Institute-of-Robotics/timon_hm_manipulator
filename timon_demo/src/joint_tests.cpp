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
  
  
  
  //ADDING FLOOR
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  moveit_msgs::CollisionObject collision_object1;
  collision_object1.header.frame_id = move_group.getPlanningFrame();


  // The id of the object is used to identify it.
  collision_object.id = "floor";
  collision_object1.id = "box"; 


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 3.0;
  primitive.dimensions[2] = 0.0;
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.2;
  primitive1.dimensions[1] = 0.2;
  primitive1.dimensions[2] = 0.3;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 0.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.11;
    geometry_msgs::Pose box_pose1;
  box_pose1.orientation.w = 0.0;
  box_pose1.position.x = 0.0;
  box_pose1.position.y = 1.06;
  box_pose1.position.z = 0.0;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_object1.primitives.push_back(primitive1);
  collision_object1.primitive_poses.push_back(box_pose1);
  collision_object1.operation = collision_object.ADD;
  

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object1);

  planning_scene_interface.addCollisionObjects(collision_objects);


  robot_state::RobotState start_state(*move_group.getCurrentState());
  move_group.setStartState(start_state);

  //JUNTAS
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = 0;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = 0;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.22; // positivo pra frente, negativo pra trás, terceiro link
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0;

  move_group.setJointValueTarget(joint_group_positions);

  geometry_msgs::Pose target_pose;      //COMANDO NO ESPAÇO DAS POSES
  // target_pose.orientation.w = 0.218820; // 0.474989;
  // target_pose.orientation.x = 0.672372; // -0.522257;
  // target_pose.orientation.y = 0.671502;  // 0.47662;
  // target_pose.orientation.z = -0.221623;  // -0.523895
  // target_pose.position.x = -0.013636; //0.000603714
  // target_pose.position.y = 0.422737; //-0.335967
  // target_pose.position.z = 0.378490; //0.305978
  // move_group.setPoseTarget(target_pose);

  // // sleep(2.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
 
  // sleep(20);                                                    
  // move_group.move();


  std::cout << move_group.getCurrentPose();

  // robot_state::RobotState start_state1(*move_group.getCurrentState());
  // move_group.setStartState(start_state1);
  // sleep(1.0);
  // target_pose.position.z = 0.45;
  // move_group.setPoseTarget(target_pose);

  // // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  // move_group.plan(my_plan);
  // move_group.execute(my_plan);

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
  if (transform.getRotation().z() >= 0.9) {    //CAIXA NA HORIZONTAL
      robot_state::RobotState start_state2(*move_group.getCurrentState());
      move_group.setStartState(start_state2);
      
      //POSE
      target_pose.orientation.w = 0.007409; // 0.474989;
      target_pose.orientation.x = -0.706906; // -0.522257;
      target_pose.orientation.y = 0.707225;  // 0.47662;
      target_pose.orientation.z = 0.007814;  // -0.523895
      target_pose.position.x = transform.getOrigin().x();
      target_pose.position.y = transform.getOrigin().y();
      target_pose.position.z = -transform.getOrigin().z();
      move_group.setGoalPositionTolerance(0.01);
      move_group.setGoalOrientationTolerance(0.3);
      move_group.setPlanningTime(10);
      move_group.setPoseTarget(target_pose);

      move_group.plan(my_plan);
      // move_group.execute(my_plan);

      //PRESS BUTTON

      // robot_state::RobotState start_state3(*move_group.getCurrentState());
      // move_group.setStartState(start_state3);

    
      //  target_pose.position.z -=0.030;       
      //  move_group.setPoseTarget(target_pose);
      //  move_group.plan(my_plan);  
      //  move_group.execute(my_plan); 

      // robot_state::RobotState start_state4(*move_group.getCurrentState());
      // move_group.setStartState(start_state4);


      //  target_pose.position.z +=0.030;       
      //  move_group.setPoseTarget(target_pose);
      // //  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      //  move_group.plan(my_plan);  
      //  move_group.execute(my_plan); 
      
  }
//   else
//   {     //CAIXA NA VERTICAL
//     robot_state::RobotState start_state4(*move_group.getCurrentState());
//    move_group.setStartState(start_state4);
// //   //POSE
//     target_pose.orientation.w =  0.456713;
//     target_pose.orientation.x = -0.539498;
//     target_pose.orientation.y =  0.540032; 
//     target_pose.orientation.z =  0.456858;
//     target_pose.position.x = transform.getOrigin().x();
//     target_pose.position.y = transform.getOrigin().z();
//     target_pose.position.z = -transform.getOrigin().y();
//     // std::cout<<"Pose alvo y: " << target_pose.position.y << "  " << "Pose alvo z: " << target_pose.position.z;

//    move_group.setGoalPositionTolerance(0.01);
//    move_group.setGoalOrientationTolerance(0.01);
//    move_group.setPlanningTime(10);
//    move_group.setPoseTarget(target_pose);
 
//   //  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
//    move_group.plan(my_plan);  
 
//    move_group.execute(my_plan); 



//     //PRESS BUTTON
//     robot_state::RobotState start_state5(*move_group.getCurrentState());
//     move_group.setStartState(start_state5);
//     target_pose.position.y +=0.031;
//     move_group.setPoseTarget(target_pose);
//     // moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
//     move_group.plan(my_plan);  
//     move_group.execute(my_plan); 

//     //GO BACK

//     // robot_state::RobotState start_state6(*move_group.getCurrentState());
//     // move_group.setStartState(start_state6);

//     // target_pose.position.y -=0.031;
//     // move_group.setPoseTarget(target_pose);
    
//     // move_group.plan(my_plan); 
//     // move_group.execute(my_plan);  
//   }
  

//BACK TO UP POSITION
  
  robot_state::RobotState start_state5(*move_group.getCurrentState());
  move_group.setStartState(start_state5);
 //JUNTAS 

 moveit::core::RobotStatePtr current_state3 = move_group.getCurrentState();
 std::vector<double> joint_group_positions3;
 current_state3->copyJointGroupPositions(joint_model_group, joint_group_positions3);
 joint_group_positions3[0] = 0;
 joint_group_positions3[1] = 0;
 joint_group_positions3[2] = 0;
 joint_group_positions3[3] = 0;
 joint_group_positions3[4] = 0;
 move_group.setJointValueTarget(joint_group_positions3);

 move_group.plan(my_plan);



  // std::cout << move_group.getCurrentPose();
  ros::shutdown(); 
  return 0;
} 

