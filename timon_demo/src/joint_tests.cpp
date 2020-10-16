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
  
  
  
  //ADDING FLOOR
  // Define a collision object ROS message.
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();
  // moveit_msgs::CollisionObject collision_object1;
  // collision_object1.header.frame_id = move_group.getPlanningFrame();


  // The id of the object is used to identify it.
  // collision_object.id = "floor";
  // collision_object1.id = "box"; 


  // // Define a box to add to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 2.0;
  // primitive.dimensions[1] = 3.0;
  // primitive.dimensions[2] = 0.0;
  // shape_msgs::SolidPrimitive primitive1;
  // primitive1.type = primitive.BOX;
  // primitive1.dimensions.resize(3);
  // primitive1.dimensions[0] = 0.4;
  // primitive1.dimensions[1] = 0.6;
  // primitive1.dimensions[2] = 0.4;

  // // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 0.0;
  // box_pose.position.x = 0.0;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = -0.011;
  //   geometry_msgs::Pose box_pose1;
  // box_pose1.orientation.w = 0.0;
  // box_pose1.position.x = 0.0;
  // box_pose1.position.y = 0.7; 
  // box_pose1.position.z = 0.0;


  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;
  // collision_object1.primitives.push_back(primitive1);
  // collision_object1.primitive_poses.push_back(box_pose1);
  // collision_object1.operation = collision_object.ADD;
  

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  // collision_objects.push_back(collision_object1);

  // planning_scene_interface.addCollisionObjects(collision_objects);


  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // move_group.setStartState(start_state);

  //JUNTAS
   moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = 0.0;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = -0.12;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.2; // positivo pra frente, negativo pra trás, terceiro link
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  // move_group.execute(my_plan);

  // geometry_msgs::Pose target_pose;      //COMANDO NO ESPAÇO DAS POSES
  // target_pose.orientation.w = 0.067434; // 0.474989;
  // target_pose.orientation.x = -0.703793; // -0.522257;
  // target_pose.orientation.y = 0.703930;  // 0.47662;
  // target_pose.orientation.z = 0.067896;  // -0.523895
  // target_pose.position.x = -0.001489; //0.000603714
  // target_pose.position.y = 0.349814; //-0.335967
  // target_pose.position.z = 0.354004; //0.305978
  // move_group.setPoseTarget(target_pose);

  // // sleep(2.0);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // move_group.plan(my_plan);

  // while (move_group.plan(my_plan).val == -1){
  //   move_group.plan(my_plan);

  // }

  ROS_INFO("Comeca timer");
  sleep(10);
  ROS_INFO("termina timer");                                                    
  // move_group.move();


  // std::cout << move_group.getCurrentPose();



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
  int i=1;
  posicao:
  try{
  // ros::Time now = ros::Time::now();
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
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = 0;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = -0.006;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.052; // positivo pra frente, negativo pra trás, terceiro link
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
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = 0;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = -0.4999;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.499; // positivo pra frente, negativo pra trás, terceiro link
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
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = 0.4;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = -0.769;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.757; // positivo pra frente, negativo pra trás, terceiro link
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
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  //COMANDO NO ESPAÇO DAS JUNTAS
  joint_group_positions[0] = -0.4;  //positivo esquerda , negativo direita , gira a base
  joint_group_positions[1] = -0.769;  //  positivo abaixo, negativo acima, segundo link
  joint_group_positions[2] = 2.757; // positivo pra frente, negativo pra trás, terceiro link
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

  ROS_INFO("Aruco Detectado");

 
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
    box_pose1.position.z = transform_fake_button_z-0.355; 
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
      target_pose.orientation.w = transform.getRotation().w(); //0.000744;//transform.getRotation().w();// // 0.474989;
      target_pose.orientation.x = transform.getRotation().x();//0.706395;//transform.getRotation().x();// // -0.522257;
      target_pose.orientation.y = transform.getRotation().y();//-0.707817;//transform.getRotation().y();//  // 0.47662;
      target_pose.orientation.z = transform.getRotation().z();// 0.000180;//transform.getRotation().z();//  // -0.523895
      target_pose.position.x = transform.getOrigin().x();  
      target_pose.position.y = transform.getOrigin().y();
      target_pose.position.z = transform.getOrigin().z();
      move_group.setGoalPositionTolerance(0.001);
      move_group.setGoalOrientationTolerance(1.0); //0.3
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
      current_state_1->copyJointGroupPositions(joint_model_group, joint_group_positions_garra);  //COMANDO NO ESPAÇO DAS JUNTAS
      // joint_group_positions[0] = 0.0;  //positivo esquerda , negativo direita , gira a base
      // joint_group_positions[1] = -0.12;  //  positivo abaixo, negativo acima, segundo link
      // joint_group_positions[2] = 2.2; // positivo pra frente, negativo pra trás, terceiro link
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

      // robot_state::RobotState start_state3(*move_group.getCurrentState());
      // move_group.setStartState(start_state3);
      //  moveit_msgs::CollisionObject box_collision;
      //  std::vector<std::string> object_ids;
      //  object_ids.push_back(box_collision.id);
      //  planning_scene_interface.removeCollisionObjects(object_ids);

       //ADDING CONSTRAINT TO END EFFECTOR
        // float or_w, or_x, or_y, or_z, x, y;
        // x = move_group.getCurrentPose().pose.position.x;
        // y = move_group.getCurrentPose().pose.position.y;
        // or_x = move_group.getCurrentPose().pose.orientation.x;
        // or_y = move_group.getCurrentPose().pose.orientation.y;
        // or_z = move_group.getCurrentPose().pose.orientation.z;
        // or_w = move_group.getCurrentPose().pose.orientation.w;
      //  moveit_msgs::OrientationConstraint ocm;
      //   ocm.link_name = "timon_link5";
      //   ocm.header.frame_id = "world";
      //   ocm.orientation.w = w;
      //   ocm.orientation.x = x;
      //   ocm.orientation.y = y;
      //   ocm.orientation.z = z;
      //   ocm.absolute_x_axis_tolerance = 0.3;
      //   ocm.absolute_y_axis_tolerance = 0.3;
      //   ocm.absolute_z_axis_tolerance = 0.3;
      //   ocm.weight = 1.0;

        // moveit_msgs::Constraints test_constraints;
        // test_constraints.orientation_constraints.push_back(ocm);
        // move_group.setPathConstraints(test_constraints);
       
      //  target_pose.orientation.w = or_w; 
      //  target_pose.orientation.x = or_x; 
      //  target_pose.orientation.y = or_y;  
      //  target_pose.orientation.z = or_z; 
      //  target_pose.position.x =x;
      //  target_pose.position.y =y;
        //  move_group.setGoalOrientationTolerance(0.6);
       target_pose.position.z -=0.174;       
       move_group.setPoseTarget(target_pose);
       move_group.plan(my_plan);  
       while (move_group.plan(my_plan).val != 1){
       move_group.plan(my_plan);
       }
      //  move_group.execute(my_plan);
      //  move_group.execute(my_plan); 

      // // robot_state::RobotState start_state4(*move_group.getCurrentState());
      // // move_group.setStartState(start_state4);
      sleep(5.0);

      //  target_pose.position.z +=0.11;       
      //  move_group.setPoseTarget(target_pose);
      // //  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      // move_group.clearPathConstraints();

      moveit::core::RobotStatePtr current_state3 = move_group.getCurrentState();
      std::vector<double> joint_group_positions3;
     current_state3->copyJointGroupPositions(joint_model_group, joint_group_positions3);
      // joint_group_positions3[0] = 0.07;
      // joint_group_positions3[1] = 0.61;
      joint_group_positions3[2] = 1.0;
      // joint_group_positions3[3] = 1.57;
      // joint_group_positions3[4] = 1.21;
     move_group.setJointValueTarget(joint_group_positions3);
       move_group.plan(my_plan);  
       while (move_group.plan(my_plan).val != 1){
       move_group.plan(my_plan);
       }
      //  move_group.execute(my_plan);
      //  move_group.execute(my_plan); 
      
  }
  else
  {     //CAIXA NA VERTICAL
  //   robot_state::RobotState start_state4(*move_group.getCurrentState());
  //  move_group.setStartState(start_state4);
//   //POSE
    ROS_INFO("Caixa Na Orientação Vertical");
    // box
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
      // target_pose.orientation.w = 0.017736; // 0.474989;
      // target_pose.orientation.x = 0.686437; // -0.522257;
      // target_pose.orientation.y = 0.726780;  // 0.47662;
      // target_pose.orientation.z =-0.016693;  // -0.523895
    target_pose.orientation.w = transform.getRotation().w();//0.55374; //0.553749;
    target_pose.orientation.x = transform.getRotation().x();//-0.41269; //-0.412694;
    target_pose.orientation.y = transform.getRotation().y();// 0.62298; //0.622989; 
    target_pose.orientation.z = transform.getRotation().z();// 0.36732; //0.367326;
    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z();
    // std::cout<<"Pose alvo y: " << target_pose.position.y << "  " << "Pose alvo z: " << target_pose.position.z;

   move_group.setGoalPositionTolerance(0.001);
   move_group.setGoalOrientationTolerance(1.0);
   move_group.setPlanningTime(25);
   move_group.setPoseTarget(target_pose);
 
  //  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
   move_group.plan(my_plan);  
      while (move_group.plan(my_plan).val != 1){
    move_group.plan(my_plan);
    }
    // move_group.execute(my_plan);
    sleep(10.0);
    // moveit_msgs::CollisionObject box_collision;
    // std::vector<std::string> object_ids;
    // object_ids.push_back(box_collision.id);
    // planning_scene_interface.removeCollisionObjects(object_ids);

  

      moveit::core::RobotStatePtr current_state_1 = move_group.getCurrentState();
      std::vector<double> joint_group_positions_garra;
      current_state_1->copyJointGroupPositions(joint_model_group, joint_group_positions_garra);  //COMANDO NO ESPAÇO DAS JUNTAS
      // joint_group_positions[0] = 0.0;  //positivo esquerda , negativo direita , gira a base
      // joint_group_positions[1] = -0.12;  //  positivo abaixo, negativo acima, segundo link
      // joint_group_positions[2] = 2.2; // positivo pra frente, negativo pra trás, terceiro link
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
    // robot_state::RobotState start_state5(*move_group.getCurrentState());
    // move_group.setStartState(start_state5);
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
    // move_group.execute(my_plan); 
    sleep(10.0);


    
    target_pose.position.y -=0.2;
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);  
    while (move_group.plan(my_plan).val != 1){
     move_group.plan(my_plan);
    }
    // move_group.execute(my_plan); 

    // moveit::core::RobotStatePtr current_state4 = move_group.getCurrentState();
    //   std::vector<double> joint_group_positions4;
    //  current_state4->copyJointGroupPositions(joint_model_group, joint_group_positions4);
    //   // joint_group_positions3[0] = 0.07;
    //   joint_group_positions4[1] = -0.7;
    //   // joint_group_positions4[2] = 1.0;
    //   // joint_group_positions3[3] = 1.57;
    //   // joint_group_positions3[4] = 1.21;
    //  move_group.setJointValueTarget(joint_group_positions4);
    //    move_group.plan(my_plan);  
    //    while (move_group.plan(my_plan).val != 1){
    //    move_group.plan(my_plan);
    //    }
    // move_group.execute(my_plan); 

     
  }
  

//BACK TO UP POSITION
  
  // robot_state::RobotState start_state5(*move_group.getCurrentState());
  // move_group.setStartState(start_state5);
 //JUNTAS 

//  moveit::core::RobotStatePtr current_state3 = move_group.getCurrentState();
//  std::vector<double> joint_group_positions3;
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



  // std::cout << move_group.getCurrentPose();
  ros::shutdown(); 
  return 0;
} 



      // joint_group_positions3[0] = 0.0167;
      // joint_group_positions3[1] = -0.749;
      // joint_group_positions3[2] = 2.667;
      // joint_group_positions3[3] = -1.54;
      // joint_group_positions3[4] = -0.95;