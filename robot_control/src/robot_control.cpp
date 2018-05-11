// template from:      https://github.com/wsnewman/learning_ros/tree/master/Part_1/example_ros_class/src

// NODE FOR HIGH LEVEL CONTROL OF DRONE

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "robot_control.h"

#include <tf2/LinearMath/Quaternion.h>

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
RobotControl::RobotControl(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of RobotControl");
    // Sleep for 2 seconds to wait for everything else to start up:
    //sleep(200);
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    
    //initialize variables here, as needed
    //val_to_remember_=0.0;
    pi = 3.14159265359;
    half_pi = pi/2;

    sonar_up_vertical_offset = 0.13;
    sonar_down_vertical_offset = -0.05;
    max_camera_range = 10; //it's actually 1.4 m but 10m cameras exist...
    camera_x_offset = 0.05;
    robot_radius = 0.2;
    robot_diameter = 0.4;
    safety_distance = 0.3;
    max_sonar_range = 6;
    radius_sq = pow(robot_radius,2);
    diameter_sq = pow(robot_radius*2,2);

    goal_point = false;
    initial_mode = true;
    orm_control = false;
    orm_turn_mode = false;
    goal_reached = false;

    spherical_matrix_degree_resolution = 6;
    spherical_matrix_height = 180/spherical_matrix_degree_resolution;
    spherical_matrix_width = 360/spherical_matrix_degree_resolution;
    M = spherical_matrix_height;//180/spherical_matrix_degree_resolution;
    N = spherical_matrix_width;

    // Set maximum translational and angular velocity
    v_max = 1; // meters per second maximum translational velocity
    omega_max = pi/4; // rad/sec maximum angular velocity

    //nearest_obstacle_distance = 5; // just initialize as something 

    // just initialize as something the following:

    // for robot radius 0.2 and cspace_rosolution 0.02 n_radius is 10
    //sphere_model[n_radius][n_radius][n_radius] = {0};
    //Cspaceframe [cspace_width] [cspace_width] = {0};

    // just initialize as something the following:
    sonar_up_limit = 6;
    range_up = 6;
    sonar_down_limit = 6;
    range_down = 0;

    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void RobotControl::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    // add more subscribers here, as needed

    occupancy_matrix_sub_ = nh_.subscribe("/spherical_matrix", 1, &RobotControl::sphericalMatrixCallback, this);

    //goal_position_sub_ = nh_.subscribe("/goal_position", 1, &RobotControl::goalPositionCallback, this);
    direction_vector_sub_ = nh_.subscribe("/target_position", 1, &RobotControl::directionVectorCallback, this);
    
    u_sol_sub_ = nh_.subscribe("/u_sol", 1, &RobotControl::usolCallback, this);
    //sonar_limits_and_ranges_sub_ = nh_.subscribe("/sonar_degree_limits", 1, &RobotControl::sonarlimitsrangesCallback, this);
    //camera_subscriber_ = nh_.subscribe("/voxelpoints", 1, &RobotControl::cameraCallback, this);
    robot_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &RobotControl::robotPositionCallback, this);
}

/*
//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void ExampleRosClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("exampleMinimalService",
                                                   &ExampleRosClass::serviceCallback,
                                                   this);  
    // add more services here, as needed
}*/

//member helper function to set up publishers;

void RobotControl::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 

    pub_desired_position_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1, true);
    pub_desired_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1, true);

}


/*void RobotControl::sonarlimitsrangesCallback(const std_msgs::Int16MultiArray::ConstPtr& input){
    sonar_up_limit = input -> data[0];
    range_up = input -> data[1];
    sonar_down_limit = input -> data[2];
    range_down = input -> data[3];

    //cout << "sonar up limit: " << sonar_up_limit << "sonar down limit: " 
    //<< sonar_down_limit << "range_up: " << range_up << "range_down: " << range_down << endl;
}*/



void RobotControl::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){
    // find smallest distance to obstacle...

    float dstride0 = matrix_msg->layout.dim[0].stride;
    float dstride1 = matrix_msg->layout.dim[1].stride;
    nearest_obstacle_distance = 100;
    for (m = 1; m< M ; m++){ // to get rid of some shit values at m=1 ...
        for (n = 0; n< N ; n++){
            //sphere_matrix[m][n] = matrix_msg->data[m*dstride1 + n];
            if(matrix_msg->data[m*dstride1 + n] < nearest_obstacle_distance && matrix_msg->data[m*dstride1 + n] != 0) {
                nearest_obstacle_distance = matrix_msg->data[m*dstride1 + n];
            }
        } 
    }
}

void RobotControl::directionVectorCallback(const geometry_msgs::Vector3ConstPtr& input) {

    direction_vector[0] = input->x;
    direction_vector[1] = input->y;
    direction_vector[2] = input->z;
/*
    goal_position.push_back(input->x);
    goal_position.push_back(input->y);
    goal_position.push_back(input->z);
    */
    goal_point = true;
    //cout<<"here 1"<< endl;
    //cout << "direction_vector: " << direction_vector << endl;
}

void RobotControl::usolCallback(const geometry_msgs::Vector3ConstPtr& input){

    u_sol[0] = input->x;
    u_sol[1] = input->y;
    u_sol[2] = input->z;

    if(u_sol[0] < 2){
        orm_control = true;
        initial_mode = false;    
    }
    else if(orm_control){
        // this happens only once when we are changing from orm control to non-orm control.
        fixed_position = robot_position;
        orm_control = false;
    }
}

void RobotControl::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input) {

    robot_position[0] = input->pose.position.x;
    robot_position[1] = input->pose.position.y;
    robot_position[2] = input->pose.position.z;

    robot_orientation[0] = input->pose.orientation.x;
    robot_orientation[1] = input->pose.orientation.y;
    robot_orientation[2] = input->pose.orientation.z;
    robot_orientation[3] = input->pose.orientation.w;

    if(goal_point){


        double distance_from_goal_point_squared = pow(direction_vector[0], 2) + 
        pow(direction_vector[1], 2) + pow(direction_vector[2], 2); 



        // if close enough to goal point just stay there .. 
        if(distance_from_goal_point_squared < 0.09) {// if less than 30 cm from goal point...
            
            cout << "goal reached!" << endl;

            if(!goal_reached){
                goal_orientation[0] = robot_orientation[0];
                goal_orientation[1] = robot_orientation[1];
                goal_orientation[2] = robot_orientation[2];
                goal_orientation[3] = robot_orientation[3];
                goal_reached = true;
            }
            set_pose.header.frame_id = "map";
            set_pose.header.stamp = ros::Time::now();
            set_pose.pose.position.x = robot_position[0] + direction_vector[0];
            set_pose.pose.position.y = robot_position[1] + direction_vector[1];
            set_pose.pose.position.z = robot_position[2] + direction_vector[2];

              // change this:
            set_pose.pose.orientation.x = 0;
            set_pose.pose.orientation.y = 0;
            set_pose.pose.orientation.z = goal_orientation[2];
            set_pose.pose.orientation.w = goal_orientation[3];

            pub_desired_position_.publish(set_pose);

        }
        else{
            goal_reached = false;

            if(orm_control){
                // do the orm control
                
                // okay let's try flying directly forward, it can be up or down, but not to the sides...
                // and just turn the robot...

                //xy_length_of_u_sol = sqrt(pow(u_sol[0],2) + pow(u_sol[1],2));

                theta = atan2(u_sol[1], u_sol[0]);

                /*acos(u_sol[0]/xy_length_of_u_sol);
                if(u_sol[1] < 0){
                        theta = -theta;
                }*/

                /*if(nearest_obstacle_distance > safety_distance){
                    v = v_max * (half_pi-abs(theta))/half_pi;
                }
                else{
                    v = v_max * nearest_obstacle_distance/safety_distance * (half_pi-abs(theta))/half_pi;
                }

                omega = omega_max * theta / half_pi;

                // all right

                geometry_msgs::TwistStamped set_velocity;

                set_velocity.header.frame_id = "base_link";
                set_velocity.header.stamp = ros::Time::now();

                set_velocity.twist.linear.x = u_sol[0]*v;
                set_velocity.twist.linear.y = 0;
                set_velocity.twist.linear.z = u_sol[2]*v;

                set_velocity.twist.angular.x = 0;
                set_velocity.twist.angular.y = 0;
                set_velocity.twist.angular.z = omega;*/


                // Let's try only turning or flying at each time, might work better:
                

                //cout << "theta: " << theta << endl;
                if(abs(theta) > 10 * pi / 180 && u_sol[2] < 0.8 && u_sol[2] > -0.8){
                    // turn the robot..
                    if(!orm_turn_mode){
                        fixed_position = robot_position;
                        orm_turn_mode = true;
                    }
                    
                    set_pose.header.frame_id = "map";
                    set_pose.header.stamp = ros::Time::now();
                    set_pose.pose.position.x = fixed_position[0];
                    set_pose.pose.position.y = fixed_position[1];
                    set_pose.pose.position.z = fixed_position[2];
                    z_turn = 1*sin(theta/2);
                    w_turn = cos(theta/2);

                    xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));


                    if(xy_length_of_direction_vector >0.2){
                        set_pose.pose.orientation.x = 0;
                        set_pose.pose.orientation.y = 0;
                        set_pose.pose.orientation.z = z_turn;
                        set_pose.pose.orientation.w = w_turn;                    
                    }
                    else{
                        set_pose.pose.orientation.x = 0;
                        set_pose.pose.orientation.y = 0;
                        set_pose.pose.orientation.z = robot_orientation[2];
                        set_pose.pose.orientation.w = robot_orientation[3]; 
                    }

                    // You can do something like this...
                    /*
                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);

                    set_pose.pose.orientation.x = q.x();
                    set_pose.pose.orientation.y = q.y();
                    set_pose.pose.orientation.z = q.z();
                    set_pose.pose.orientation.w = q.w();
                    */

                    pub_desired_position_.publish(set_pose);

                    /*omega = omega_max * theta / half_pi;
                    set_velocity.twist.linear.x = 0;
                    set_velocity.twist.linear.y = 0;
                    set_velocity.twist.linear.z = 0;
                    set_velocity.twist.angular.x = 0;
                    set_velocity.twist.angular.y = 0;
                    set_velocity.twist.angular.z = omega;*/
                }
                else{

                    geometry_msgs::TwistStamped set_velocity;
                    set_velocity.header.frame_id = "base_link";
                    set_velocity.header.stamp = ros::Time::now();

                    if(nearest_obstacle_distance > safety_distance){
                        v = v_max * (half_pi-abs(theta))/half_pi;
                    }
                    else{
                        v = v_max * nearest_obstacle_distance/safety_distance * (half_pi-abs(theta))/half_pi;
                    }
                    if(distance_from_goal_point_squared < 4){
                        // lower speed because getting close to target
                         v *= sqrt(distance_from_goal_point_squared)/2+0.05;
                    }
                    cout << "nearest_obstacle_distance" << nearest_obstacle_distance << endl; 
                    cout << "u_sol in control: " << u_sol  << "and v is: " << v << endl;

                    Vector3d robot_orientation_v(robot_orientation[0], robot_orientation[1], robot_orientation[2]);
                    float robot_orientation_w = robot_orientation[3];

                    Vector3d u_sol_global = u_sol + 2*robot_orientation_w*cross(robot_orientation_v, u_sol) +
                    2*cross(robot_orientation_v, cross(robot_orientation_v, u_sol));


                    set_velocity.twist.linear.x = u_sol_global[0]*v;
                    set_velocity.twist.linear.y = u_sol_global[1]*v;
                    set_velocity.twist.linear.z = u_sol_global[2]*v;

                    set_velocity.twist.angular.x = 0;
                    set_velocity.twist.angular.y = 0;
                    set_velocity.twist.angular.z = 0;
                    pub_desired_velocity_.publish(set_velocity);

                    orm_turn_mode = false;

                    //cout << "here :>)" <<endl;
                }
            }
            else{
                // keep on same spot and turn towards target point.

                //direction_vector = goal_position - robot_position;

                //cout<<"here 2"<< endl;
                //cout << "robot_vector: " << robot_position[0] << " " << robot_position[1] <<" " << robot_position[2] << endl;
                //cout << "goal_position: " << goal_position[0] << " " << goal_position[1] <<" " << goal_position[2] << endl;

                //cout << "direction_vector: " << direction_vector[0] << " " << direction_vector[1] <<" " << direction_vector[2] << endl;

                xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));

                set_pose.header.frame_id = "map";
                set_pose.header.stamp = ros::Time::now();

                theta = atan2(direction_vector[1], direction_vector[0]);/*acos(direction_vector[0]/xy_length_of_direction_vector);
                if(direction_vector[1] < 0){
                        theta = -theta;
                }*/

                /*cout << "theta is: " << theta << endl;
                cout << "direction_vector is" << direction_vector << endl;*/
                //and we want to turn around z axis so our quaternion coordinates become:
                //double x= 0;
                //double y = 0;
                z_turn = 1*sin(theta/2);
                w_turn = cos(theta/2);

                
                if(initial_mode){// only do this if we haven't started moving yet.. 
                    set_pose.pose.position.x = 0;
                    set_pose.pose.position.y = 0;
                    set_pose.pose.position.z = 1;
                    //}
                }
                else{
                    set_pose.pose.position.x = fixed_position[0];
                    set_pose.pose.position.y = fixed_position[1];
                    set_pose.pose.position.z = fixed_position[2];
                }

                // otherwise keep same position if we have no u_sol .. 

                if(xy_length_of_direction_vector >0.2){
                    set_pose.pose.orientation.x = 0;
                    set_pose.pose.orientation.y = 0;
                    set_pose.pose.orientation.z = z_turn;
                    set_pose.pose.orientation.w = w_turn;                    
                }
                else{
                    set_pose.pose.orientation.x = 0;
                    set_pose.pose.orientation.y = 0;
                    set_pose.pose.orientation.z = robot_orientation[2];
                    set_pose.pose.orientation.w = robot_orientation[3]; 
                }


                pub_desired_position_.publish(set_pose);

                orm_turn_mode = false;
            }
        }
    }
}


Vector3d RobotControl::cross(const Vector3d & vec1, const Vector3d & vec2){
    Vector3d cross_product;
    cross_product[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    cross_product[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    cross_product[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    
    return ( cross_product );
}

int main(int argc, char** argv)
{

// ROS set-ups:
    ros::init(argc, argv, "robot_control"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type RobotControl");
    RobotControl robotControl(&nh);  //instantiate an RobotControl object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");

    //Wait a second, maybe do all the work here instead... 

    ros::spin();
    return 0;
}
