// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef RECORD_DATA_H_
#define RECORD_DATA_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

// To receive occupancy matrix:
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/MultiArrayDimension.h>

#include <nav_msgs/Odometry.h>


#include <iostream>
#include <fstream>
#include <ctime>

//for eigen vectors:
#include <Eigen/Dense>
using namespace Eigen;

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h> // uses the "Trigger.srv" message defined in ROS

using namespace std;
//template <typename T>

// define a class, including a constructor, member variables and member functions
class RecordData
{
public:
    RecordData(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber robot_position_sub; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber robot_velocity_sub;
    ros::Subscriber sphere_matrix_sub;
    ros::Subscriber robot_local_velocity_sub;
    
    ros::Subscriber velocity_command_sub;
    //ros::Subscriber nearest_obstacle_sub;

    string filename;

    ros::ServiceServer start_recording_service_;
    ros::ServiceServer stop_recording_service_;
    //ros::Publisher  minimal_publisher_;
    
    float robot_orientation [4];
    int m,n,M,N, nearest_obstacle_distance_m, nearest_obstacle_distance_n;
    int spherical_matrix_height, spherical_matrix_width, spherical_matrix_degree_resolution;

    Vector3d robot_position, robot_translational_velocity, robot_angular_velocity, local_robot_translational_velocity, local_robot_angular_velocity, command_translational_velocity, command_angular_velocity;
    double nearest_obstacle_distance;
    bool start_recording;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    //void initializePublishers();
    void initializeServices();
    void initializeFileName();

    template <typename T> string ToString(T val);

    void sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg);
    void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input);
    void robotVelocityCallback(const geometry_msgs::TwistStampedConstPtr& input);
    void robotLocalVelocityCallback(const nav_msgs::OdometryConstPtr& input);

    void velocityCommandCallback(const geometry_msgs::TwistStampedConstPtr& input);

    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for example service
    bool startRecordingServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool stopRecordingServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef