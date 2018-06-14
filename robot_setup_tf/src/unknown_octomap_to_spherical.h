// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef OCTOMAP_TO_SPHERICAL_H_
#define OCTOMAP_TO_SPHERICAL_H_

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

//for pcl point cloud: 
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

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

// To receive occupancy matrix:
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/MultiArrayDimension.h>

#include <cmath>
#include "math.h"

//#include <octomap_msgs/binaryMsgToMap.h>

//#include "point3d.h"
#include <algorithm>

#include "sensor_msgs/PointCloud2.h"

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>


//for eigen vectors:
#include <Eigen/Dense>
using namespace Eigen;
using namespace octomap;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// define a class, including a constructor, member variables and member functions
class OctomapToSpherical
{
public:
    OctomapToSpherical(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;

    ros::ServiceClient bbx_service_client;

    ros::Subscriber robot_position_sub_;
    ros::Subscriber octomap_sub_;

    ros::Publisher pub;
    ros::Publisher pub_matrix;
    ros::Publisher pub_subgoal_matrix;
    ros::Publisher pub_cloud;
    ros::Publisher pub_init_cloud;
    ros::Publisher pub_subgoal_cloud;
    ros::Publisher pub_points_checked_cloud;



    Vector3d goal_position, robot_position;
    float robot_orientation [4];
    double box_width_m;
    int M,N;

    bool sphere_model [100][100][100];//[200] [200] [200]; //box_width_m/octomap_resolution e.g. 10 m / 0.05 m = 200  [50][50][50];//

    double robot_radius;
//    double robot_diameter;
    double safety_distance;

    double laser_vertical_offset;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    double resolution, pi, deg_resolution, half_box_width_m_sq, half_box_width_m;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber

    void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input);

    Vector3d cross(const Vector3d & vec1, const Vector3d & vec2);

    void octoMapCallback(const octomap_msgs::OctomapConstPtr& msg);
    //prototype for callback for example service
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);


    float dot(const Vector3d & vec1, const Vector3d & vec2);

    void initializeOctomap();
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef