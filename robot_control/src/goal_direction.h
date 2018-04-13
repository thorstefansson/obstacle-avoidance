// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef GOAL_DIRECTION_H_
#define GOAL_DIRECTION_H_

//some generically useful stuff to include...
#include <math.h>

//following three perhaps not needed
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "math.h"

#include <ros/ros.h> //ALWAYS need to include this

#include "sensor_msgs/Range.h"
#include "sensor_msgs/PointCloud2.h"
//maybe use this: ?
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"

//for eigen vectors:
#include <Eigen/Dense>
using namespace Eigen;

// To receive occupancy matrix:
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/MultiArrayDimension.h>

ros::Publisher pub;

//using namespace cmath
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// define a class, including a constructor, member variables and member functions
class GoalDirection
{
public:
    GoalDirection(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ros::Subscriber robot_position_sub_;
    ros::Subscriber goal_position_sub_;
    ros::Subscriber occupancy_matrix_sub_;
    ros::Subscriber sonar_limits_and_ranges_sub_;
    ros::Subscriber camera_subscriber_;

    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;
    ros::Publisher pub_desired_position_;
    ros::Publisher pub_subgoal_cloud;
    ros::Publisher pub;
    ros::Publisher pub_Cspace;
    ros::Publisher pub_selected_subgoal;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    //double goal_position [3];
    //double robot_position [3];
    //vector<double> goal_position;
    //vector<double>robot_position;
    /*
    geometry_msgs::Vector3 goal_position;
    geometry_msgs::Vector3 robot_position;
    geometry_msgs::Vector3 direction_vector;*/

    Vector3d goal_position, robot_position, direction_vector, subgoal_vector;

    //float goal_position[3], robot_position[3], direction_vector[3];

    float robot_orientation [4];
    //Vector3d direction;

    pcl::PointCloud<pcl::PointXYZ> camera_cloud;
    
    //int M = 60;
    //int N = 120;
    double sphere_matrix [60][120];

    double val_to_remember_; // member variables will retain their values even as callbacks come and go

    double pi;

    double sonar_up_vertical_offset;
    double sonar_down_vertical_offset;
    double max_camera_range;
    double robot_radius;
    double robot_diameter;
    double safety_distance;
    double max_sonar_range;
    float radius_sq;
    float diameter_sq;
    double cspace_resolution;
    int cspace_width, cspace_half_width;
    int Cspaceframe[20][20], n_radius;
    int Cspaceframe_activepoints;
    int sphere_model[10][10][10];
    //int Cspace [][20][20];
    double subgoal_matrix[60][120];// = {0};
    int spherical_matrix_height, spherical_matrix_width;


    int sonar_up_limit, sonar_down_limit, range_up, range_down;

    int countx, county, countz;
    
    double camera_x_offset;

    typedef std::pair<double,int> distance_and_index;
    struct comparator{
        bool operator() ( const distance_and_index& l, const distance_and_index& r) const
        { return l.first < r.first; }
    };

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber   !!!!!!!!!!!!!!!!! 
    //void cameraCallback (const sensor_msgs::PointCloud2ConstPtr& input);

    void goalPositionCallback(const geometry_msgs::Vector3ConstPtr& input);
    void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input);
    void sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg);
    //double [60][120] sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg);
    void sonarlimitsrangesCallback(const std_msgs::Int16MultiArray::ConstPtr& input);
    void cameraCallback (const sensor_msgs::PointCloud2ConstPtr& input);
    bool isReachable(const Vector3d & direction);//geometry_msgs::Vector3 direction);//const geometry_msgs::Vector3ConstPtr& direction);
    float CylTest_CapsFirst(const Vector3d & dir_vec, float lengthsq, float radius_sq, float pdx, float pdy, float pdz);
    Vector3d cross(const Vector3d & vec1, const Vector3d & vec2);
    void initializeSphere();
    //bool comparator ( const distance_and_index& l, const distance_and_index& r);
    struct comparator;
    //void addObstacleSphere(int obstacle_point_center_x, int obstacle_point_center_y, int obstacle_point_center_z, int cspace_length);
    //const geometry_msgs::Vector3 & dir_vec, float lengthsq, float radius_sq, float pdx, float pdy, float pdz);

    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef