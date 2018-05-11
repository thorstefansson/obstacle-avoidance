// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

//some generically useful stuff to include...
#include <math.h>

//following three perhaps not needed
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "math.h"


#include <algorithm>

#include <ros/ros.h> //ALWAYS need to include this

#include "sensor_msgs/PointCloud2.h"
//maybe use this: ?

#include <iostream>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//for eigen vectors:
#include <Eigen/Dense>
using namespace Eigen;

// To receive occupancy matrix:
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/MultiArrayDimension.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher pub;

//using namespace cmath

using namespace std;



// define a class, including a constructor, member variables and member functions
class RobotControl
{
public:
    RobotControl(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details !!!!!!!!!!!!!!!!!!!!!!
    ros::Subscriber robot_position_sub_;
    ros::Subscriber direction_vector_sub_;
    ros::Subscriber sonar_limits_and_ranges_sub_;
    ros::Subscriber occupancy_matrix_sub_;
    ros::Subscriber u_sol_sub_;
    //ros::Subscriber u_targ_sub_

    //ros::Subscriber camera_subscriber_;

    //ros::ServiceServer minimal_service_;
    //ros::Publisher  minimal_publisher_;
    ros::Publisher pub_desired_position_;
    ros::Publisher pub_desired_velocity_;
    
    double val_from_subscriber_; //example member variable: better than using globals; 
                                //convenient way to pass data from a subscriber to other member functions
    //double goal_position [3];
    //double robot_position [3];
    //vector<double> goal_position;
    //vector<double>robot_position;
    /*
    geometry_msgs::Vector3 goal_position;
    geometry_msgs::Vector3 robot_position;
    geometry_msgs::Vector3 direction_vector;*/

    Vector3d direction_vector, robot_position, subgoal_vector, u_sol, fixed_position;

    //float goal_position[3], robot_position[3], direction_vector[3];

    float robot_orientation [4];
    //Vector3d direction;

    double val_to_remember_; // member variables will retain their values even as callbacks come and go

    double pi, half_pi;
    double sonar_up_vertical_offset;
    double sonar_down_vertical_offset;
    double max_camera_range;
    double robot_radius;
    double robot_diameter;
    double safety_distance;
    double max_sonar_range;
    float radius_sq;
    float diameter_sq;

    tf2::Quaternion q, q2;
    double roll, pitch, yaw;


    double z_turn, w_turn, xy_length_of_direction_vector, xy_length_of_u_sol;

    geometry_msgs::PoseStamped set_pose;

    int spherical_matrix_height, spherical_matrix_width;
    int M,N, spherical_matrix_degree_resolution, m, n;
    float nearest_obstacle_distance;
    double sphere_matrix [30][60];

    double v_max, omega_max, v, omega, theta;

    int sonar_up_limit, sonar_down_limit, range_up, range_down;

    bool goal_point, initial_mode, orm_control, orm_turn_mode, goal_reached;

    float goal_orientation[4];
    
    double camera_x_offset;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, 
                                    //publishers and services
    void initializePublishers();
    //void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber   !!!!!!!!!!!!!!!!! 

    void directionVectorCallback(const geometry_msgs::Vector3ConstPtr& input);
    void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input);
    void sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg);
    void usolCallback(const geometry_msgs::Vector3ConstPtr& input);
    //void utargCallback(const geometry_msgs::Vector3ConstPtr& input);
    //void sonarlimitsrangesCallback(const std_msgs::Int16MultiArray::ConstPtr& input);
    Vector3d cross(const Vector3d & vec1, const Vector3d & vec2);

    //const geometry_msgs::Vector3 & dir_vec, float lengthsq, float radius_sq, float pdx, float pdy, float pdz);

    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef
