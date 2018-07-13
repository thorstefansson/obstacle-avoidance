// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef OCTOMAP_TIME_MEASURING_H_
#define OCTOMAP_TIME_MEASURING_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h> // uses the "Trigger.srv" message defined in ROS

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<octomap/OcTreeBase.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <iostream>
#include <fstream>
#include <ctime>

// To receive occupancy matrix:
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/MultiArrayDimension.h>

#include <cmath>
#include "math.h"

//#include <octomap_msgs/binaryMsgToMap.h>

//#include "point3d.h"
#include <algorithm>



//for eigen vectors:
#include <Eigen/Dense>
using namespace Eigen;
using namespace octomap;
using namespace std;


// define a class, including a constructor, member variables and member functions
class OctomapTimeMeasuring
{
public:
    OctomapTimeMeasuring(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;

    ros::Subscriber octomap_sub_;


    string filename_octomap;

    double time_between_octomap_callbacks, previous_time_of_octomap_callback;

    template <typename T> string ToString(T val);


    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    // void initializePublishers();
    // void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber

    void octoMapCallback(const octomap_msgs::OctomapConstPtr& msg);
    //prototype for callback for example service

    void initializeFileName();

}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef