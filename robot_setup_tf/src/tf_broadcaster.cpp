#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.0, 0.08)),
        ros::Time::now(),"base_link", "laser_link"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(-0.5,0.5,-0.5,0.5), tf::Vector3(0.05, 0.0, 0.13)),
        ros::Time::now(),"base_link", "camera_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, -0.7071068,0, 0.7071068), tf::Vector3(-0.05, 0.0, 0.13)),
        ros::Time::now(),"base_link", "sonar_up_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0.7071068,0, 0.7071068), tf::Vector3(-0.05, 0.0, -0.05)),
        ros::Time::now(),"base_link", "sonar_down_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0.7071068,0, 0.7071068), tf::Vector3(0, 0.0, -0.05)),
        ros::Time::now(),"base_link", "teraranger"));


    r.sleep();
  }
}
