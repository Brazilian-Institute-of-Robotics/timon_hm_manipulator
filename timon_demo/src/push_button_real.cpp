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
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

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
  
  ros::Time start_time = ros::Time::now();
  
  //JUNTAS
   moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO DOMÍNIO DAS JUNTAS
  joint_group_positions[0] = 0.0;  
  joint_group_positions[1] = -0.12;  
  joint_group_positions[2] = 2.2; 
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  //move_group.execute(my_plan)

  sleep(5);

  // Define variables for xyz for the box
  double transform_fake_button_x;
  double transform_fake_button_y;
  double transform_fake_button_z;
  double transform_fake_button_or_x;
  double transform_fake_button_or_y;
  double transform_fake_button_or_z;
  double transform_fake_button_or_w;
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object; //floor
  collision_object.header.frame_id = move_group.getPlanningFrame();
  moveit_msgs::CollisionObject collision_object1; //box
  collision_object1.header.frame_id = move_group.getPlanningFrame();
  // The id of the object is used to identify it.
  collision_object.id = "floor";
  collision_object1.id = "box"; 
  // Define a box to add to the world.
  // floor
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 3.0;
  primitive.dimensions[2] = 0.0;
  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 0.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;


  tf::TransformListener listener;
  //GETTING FAKE BUTTON POSE
  tf::StampedTransform transform, transform_button_;
  int i=2;
  posicao:
  try{
  std::cout << i;
  i = i+1;
  listener.waitForTransform("/invisible_link", "/fake_botao", 
                           ros::Time(0), ros::Duration(2.0));
  listener.lookupTransform("/invisible_link", "/fake_botao",
                           ros::Time(0), transform);

  transform_fake_button_x = transform.getOrigin().x();
  transform_fake_button_y = transform.getOrigin().y();
  transform_fake_button_z = transform.getOrigin().z();
  transform_fake_button_or_x = transform.getRotation().x();
  transform_fake_button_or_y = transform.getRotation().y();
  transform_fake_button_or_z = transform.getRotation().z();
  transform_fake_button_or_w = transform.getRotation().w();

  }
  catch (tf::TransformException &ex) {

  if(i == 1){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); 
  joint_group_positions[0] = 0;  
  joint_group_positions[1] = -0.006;  
  joint_group_positions[2] = 2.052; 
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  // move_group.execute(my_plan);
  sleep (5.0);
  }

  if(i == 2){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); 
  joint_group_positions[0] = 0;  
  joint_group_positions[1] = -0.4999;  
  joint_group_positions[2] = 2.499; 
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  // move_group.execute(my_plan);
  sleep (5.0);
  }

  if(i == 3){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); 
  joint_group_positions[0] = 0.4;  
  joint_group_positions[1] = -0.769;  
  joint_group_positions[2] = 2.757; 
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  // move_group.execute(my_plan);
  sleep (5.0);
  }

  if(i == 4){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); 
  joint_group_positions[0] = -0.4;  
  joint_group_positions[1] = -0.769;  
  joint_group_positions[2] = 2.757; 
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  // move_group.execute(my_plan);
  i=0;
  sleep (5.0);
  }

  goto posicao;

  }
  //GETTING BUTTON POSE
  try{
  // ros::Time now = ros::Time::now();
  listener.waitForTransform("/invisible_link", "/botao_press", 
                           ros::Time(0), ros::Duration(2.0));
  listener.lookupTransform("/invisible_link", "/botao_press",
                           ros::Time(0), transform_button_);
  std::cout<<"Pose do botão: x: "<< transform_button_.getOrigin().x()<<
                           " y: "<< transform_button_.getOrigin().y()<<
                           " z: "<< transform_button_.getOrigin().z();   

  }
  catch (tf::TransformException &ex) {
  i=0;
  goto posicao;
  }

  ROS_INFO("Aruco Detectado");
  ros::Duration delta_t1 = ros::Time::now() - start_time;
  double delta_t1_sec = delta_t1.toSec();

  std::cout<< "Tempo de busca: " << delta_t1_sec;

 
  if (transform.getRotation().x() <= 0.5) { 
    ROS_INFO("Caixa na Horizontal");
    // box
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.2;
    primitive1.dimensions[1] = 0.3;
    primitive1.dimensions[2] = 0.2;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose1;
    box_pose1.position.x = transform_fake_button_x ;
    box_pose1.position.y = transform_fake_button_y;
    box_pose1.position.z = transform_fake_button_z-0.354; 
    box_pose1.orientation.w = transform_fake_button_or_w;
    box_pose1.orientation.x = transform_fake_button_or_x;
    box_pose1.orientation.y = transform_fake_button_or_y;
    box_pose1.orientation.z = transform_fake_button_or_z;
    //Collision box
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

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = transform.getRotation().w(); 
    target_pose.orientation.x = transform.getRotation().x();
    target_pose.orientation.y = transform.getRotation().y();
    target_pose.orientation.z = transform.getRotation().z();
    target_pose.position.x = transform.getOrigin().x();  
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z();
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.35); 
    move_group.setPlanningTime(25);
    move_group.setPoseTarget(target_pose);

    move_group.plan(my_plan);
    while (move_group.plan(my_plan).val != 1){
      move_group.plan(my_plan);
    }
    // move_group.execute(my_plan); 
      

    sleep(16.0);

    moveit::core::RobotStatePtr current_state_1 = move_group.getCurrentState();
    std::vector<double> joint_group_positions_garra;
    current_state_1->copyJointGroupPositions(joint_model_group, joint_group_positions_garra); 
    joint_group_positions_garra[3] = 1.57;
    joint_group_positions_garra[4] = 1.21;
    move_group.setJointValueTarget(joint_group_positions_garra);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    while(move_group.plan(my_plan).val == -1){
      move_group.plan(my_plan);
    }
    sleep(11.0);
      // move_group.execute(my_plan);

      //PRESS BUTTON

    target_pose.position.z -=0.175;       
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1; 
    move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    sleep(5.0);
    
    geometry_msgs::Pose pose_real;
    pose_real = move_group.getCurrentPose().pose;
    std :: cout << "Pose Real: " << pose_real.position; 
  

    target_pose.position.z +=0.11;       
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);
    while (move_group.plan(my_plan).val != 1){
      move_group.plan(my_plan);
    }
      
  }
  else
  { // box
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.2;
    primitive1.dimensions[1] = 0.3;
    primitive1.dimensions[2] = 0.2;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose1;
        box_pose1.orientation.w = transform_fake_button_or_w;
    box_pose1.orientation.x = transform_fake_button_or_x;
    box_pose1.orientation.y = transform_fake_button_or_y;
    box_pose1.orientation.z = transform_fake_button_or_z;
    box_pose1.position.x = transform_fake_button_x -0.1;
    box_pose1.position.y = transform_fake_button_y+0.318;
    box_pose1.position.z = transform_fake_button_z - 0.1;  
    //Collision box
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

    geometry_msgs::Pose target_pose;

    target_pose.orientation.w = transform.getRotation().w();
    target_pose.orientation.x = transform.getRotation().x();
    target_pose.orientation.y = transform.getRotation().y();
    target_pose.orientation.z = transform.getRotation().z();
    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z();
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(1.0);
    move_group.setPlanningTime(25);
    move_group.setPoseTarget(target_pose); 
    move_group.plan(my_plan);  
      while (move_group.plan(my_plan).val != 1){
    move_group.plan(my_plan);
    }
    // move_group.execute(my_plan);
    sleep(10.0);
    moveit::core::RobotStatePtr current_state_1 = move_group.getCurrentState();
    std::vector<double> joint_group_positions_garra;
    current_state_1->copyJointGroupPositions(joint_model_group, joint_group_positions_garra); 
    joint_group_positions_garra[3] = -2.12;
    joint_group_positions_garra[4] = 1.21;
    move_group.setJointValueTarget(joint_group_positions_garra);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    while(move_group.plan(my_plan).val == -1){
      move_group.plan(my_plan);
    }
    sleep(10.0); 

    //PRESS BUTTON
    move_group.setGoalOrientationTolerance(0.9);
    float or_w, or_x, or_y, or_z, x, y;
    or_x = move_group.getCurrentPose().pose.orientation.x;
    or_y = move_group.getCurrentPose().pose.orientation.y;
    or_z = move_group.getCurrentPose().pose.orientation.z;
    or_w = move_group.getCurrentPose().pose.orientation.w;
    target_pose.orientation.w = or_w; 
    target_pose.orientation.x = or_x; 
    target_pose.orientation.y = or_y;  
    target_pose.orientation.z = or_z;  
    target_pose.position.y +=0.172;
    target_pose.position.z += 0.018;
    move_group.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
    move_group.plan(my_plan);  
    while (move_group.plan(my_plan).val != 1){
      move_group.plan(my_plan);
      }  
    sleep(10.0);

    target_pose.position.y -=0.2;
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);  
    while (move_group.plan(my_plan).val != 1){
     move_group.plan(my_plan);
    }
    // move_group.execute(my_plan); 
    
}
  

//BACK TO HOME POSITION
  
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
joint_group_positions[0] = 0.0;
joint_group_positions[1] = -0.12;
joint_group_positions[2] = 2.2;
joint_group_positions[3] = 0;
joint_group_positions[4] = 0.0;
move_group.setJointValueTarget(joint_group_positions);

move_group.plan(my_plan);  
while (move_group.plan(my_plan).val != 1){
  move_group.plan(my_plan);
  }
// move_group.execute(my_plan); 
  sleep(5.0);
  ros::Duration delta_t = ros::Time::now() - start_time;
  double delta_t_sec = delta_t.toSec();

  std::cout<< "Tempo de missão: " << delta_t_sec;

  ros::shutdown(); 
  return 0;
} 
