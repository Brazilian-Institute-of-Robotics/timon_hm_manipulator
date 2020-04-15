#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    while (nh.ok()){
        tf::StampedTransform transform;
        try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/botao", "/timon_link9", 
                                now, ros::Duration(1.0));
        listener.lookupTransform("/botao", "/timon_link9",
                                now, transform);
        }
        catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
        }
    std::cout << transform.getOrigin().z() << std::endl;
    rate.sleep();
     ROS_INFO("tf is good");
    }
    // std::cout<<transform;
    return 0;

};