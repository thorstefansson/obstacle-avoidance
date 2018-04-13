// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef SENSORS_TO_SPHERICAL_H_
#define SENSORS_TO_SPHERICAL_H_

//some generically useful stuff to include...
#include <math.h>

//following three perhaps not needed
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed

//#include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

#include "sensor_msgs/Range.h"
#include "sensor_msgs/PointCloud2.h"
//maybe use this: ?
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>

//for publishing matrix:
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

ros::Publisher pub;

//using namespace cmath
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// define a class, including a constructor, member variables and member functions
class SensorsToSpherical
{
public:
    SensorsToSpherical(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber camera_subscriber_;
    ros::Subscriber sonar_up_subscriber_;
    ros::Subscriber sonar_down_subscriber_;
    ros::Subscriber lidar_cloud_subscriber_;


    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;
    
    //uncomment following for viewing cloud in rviz:
    ros::Publisher pub;
    
    ros::Publisher pub_matrix;
    ros::Publisher pub_sonar_degree_limits_and_ranges;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    pcl::PointCloud<pcl::PointXYZ> lidarcloud;
    double range_up;
    double range_down;
    //double sonar_up_offset [3];
    //double sonar_down_offset [3];
    double sonar_up_vertical_offset;
    double sonar_down_vertical_offset;

    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    double sin22komma5;
    double pi;
    double max_camera_range;
    double robot_radius;
    double safety_distance;
    double max_sonar_range;
    double camera_x_offset;
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    
    void cameraCallback (const sensor_msgs::PointCloud2ConstPtr& input);
    void sonarupCallback (const sensor_msgs::RangeConstPtr& input);
    void sonardownCallback (const sensor_msgs::RangeConstPtr& input);
    void lidarcloudCallback (const sensor_msgs::PointCloud2ConstPtr& input);

    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef