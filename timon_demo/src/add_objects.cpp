#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_objects");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Define publisher to update work scene
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define floor plane
    shape_msgs::SolidPrimitive floor;
    floor.type = floor.BOX;
    floor.dimensions.resize(3);
    floor.dimensions[0] = 2.0; // x
    floor.dimensions[1] = 3.0; // y
    floor.dimensions[2] = 0.03; // z
    // Define floor position
    geometry_msgs::Pose floor_pose;
    floor_pose.orientation.w = 0.0;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.03;

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "floor";
    collision_objects.header.frame_id = "world";
    collision_objects.primitives.push_back(floor);
    collision_objects.primitive_poses.push_back(floor_pose);
    collision_objects.operation = collision_objects.ADD;

    // Define box components
    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.5;
    box.dimensions[1] = 0.75;
    box.dimensions[2] = 0.65;

    // shape_msgs::SolidPrimitive maker2;
    // maker2.type = maker2.BOX;
    // maker2.dimensions.resize(3);
    // maker2.dimensions[0] = 0.4;
    // maker2.dimensions[1] = 0.42;
    // maker2.dimensions[2] = 0.02;

    //Define positions
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.7;
    box_pose.position.z = 0.0;

    // geometry_msgs::Pose maker1R_pose;
    // maker1R_pose.orientation.w = 1.0;
    // maker1R_pose.position.x = -0.73;
    // maker1R_pose.position.y = 0.22;
    // maker1R_pose.position.z = 0.25;

    // geometry_msgs::Pose maker2D_pose;
    // maker2D_pose.orientation.w = 1.0;
    // maker2D_pose.position.x = -0.80;
    // maker2D_pose.position.y = 0.0;
    // maker2D_pose.position.z = 0.15;

    // geometry_msgs::Pose maker2U_pose;
    // maker2U_pose.orientation.w = 1.0;
    // maker2U_pose.position.x = -0.80;
    // maker2U_pose.position.y = 0.0;
    // maker2U_pose.position.z = 0.35;

    //Define collision objects
    moveit_msgs::CollisionObject box_collision;
    box_collision.id = "box";
    box_collision.header.frame_id = "world";
    box_collision.primitives.push_back(box);
    box_collision.primitive_poses.push_back(box_pose);
    box_collision.operation = collision_objects.ADD;   

    // moveit_msgs::CollisionObject maker1R_collision;
    // maker1R_collision.id = "maker1R";
    // maker1R_collision.header.frame_id = "world";
    // maker1R_collision.primitives.push_back(maker1);
    // maker1R_collision.primitive_poses.push_back(maker1R_pose);
    // maker1R_collision.operation = collision_objects.ADD;   

    // moveit_msgs::CollisionObject maker2D_collision;
    // maker2D_collision.id = "maker2D";
    // maker2D_collision.header.frame_id = "world";
    // maker2D_collision.primitives.push_back(maker2);
    // maker2D_collision.primitive_poses.push_back(maker2D_pose);
    // maker2D_collision.operation = collision_objects.ADD;   

    // moveit_msgs::CollisionObject maker2U_collision;
    // maker2U_collision.id = "maker2U";
    // maker2U_collision.header.frame_id = "world";
    // maker2U_collision.primitives.push_back(maker2);
    // maker2U_collision.primitive_poses.push_back(maker2U_pose);
    // maker2U_collision.operation = collision_objects.ADD;   

    // // Define A Cylinder
    // shape_msgs::SolidPrimitive object1;
    // object1.type = object1.CYLINDER;
    // object1.dimensions.resize(2);
    // object1.dimensions[0] = 0.12; // height
    // object1.dimensions[1] = 0.02; // radius
    // // Define cylinder position
    // geometry_msgs::Pose object1_pose;
    // object1_pose.orientation.w = 1.0;
    // object1_pose.position.x = -0.75; //-0.85
    // object1_pose.position.y = 0.0; 
    // object1_pose.position.z = 0.21;  //0.2


    // // Define collision2 objects
    // moveit_msgs::CollisionObject collision_objects2;
    // collision_objects2.id = "object";
    // collision_objects2.header.frame_id = "world";
    // collision_objects2.primitives.push_back(object1);
    // collision_objects2.primitive_poses.push_back(object1_pose);
    // collision_objects2.operation = collision_objects.ADD;


    // Add all objects to environment
    ROS_INFO("Adding the all objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
    // work_scene.world.collision_objects.push_back(attached_objects.object);
    work_scene.world.collision_objects.push_back(collision_objects);
    // work_scene.world.collision_objects.push_back(box_collision);
    // work_scene.world.collision_objects.push_back(maker1R_collision);
    // work_scene.world.collision_objects.push_back(maker2D_collision);
    // work_scene.world.collision_objects.push_back(maker2U_collision);
    // work_scene.world.collision_objects.push_back(collision_objects2);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::WallDuration(1).sleep();


    ros::shutdown();
    return 0;
}