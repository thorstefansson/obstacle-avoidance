#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>




void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z ) );
  //tf::Quaternion q; var z + 0.067
  //q.setRPY(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

  // for visualizing mavros stuff in rviz:
  // we need to define transform between world and map
  transform.setOrigin( tf::Vector3(0,0,0) );
  transform.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

}

int main(int argc, char** argv){
  ros::init(argc, argv,  "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/mavros/local_position/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
