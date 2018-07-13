#ifndef TERARANGER_TO_POINT_CLOUD_H_
#define TERARANGER_TO_POINT_CLOUD_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include "sensor_msgs/Range.h"
#include "sensor_msgs/PointCloud2.h"
//maybe use this: ?
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// define a class, including a constructor, member variables and member functions
class TeraRangerToPointCloud
{
public:
    TeraRangerToPointCloud(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;

    ros::Publisher pub_teraranger_cloud;

    ros::Subscriber teraranger_subscriber_;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber

    void teraRangerCallback (const sensor_msgs::RangeConstPtr& input);
    //prototype for callback for example service
    //bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef