//example_ros_class.cpp:
//wsn, Jan 2016
//illustrates how to use classes to make ROS nodes
// constructor can do the initialization work, including setting up subscribers, publishers and services
// can use member variables to pass data from subscribers to other member functions

// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo example_class_output_topic
// rostopic pub -r 4 example_class_input_topic std_msgs/Float32 2.0
// rosservice call example_minimal_service     // sends a trigger signal; don't need a request argument


// this header incorporates all the necessary #include files and defines the class "RecordData"
#include "record_data.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
RecordData::RecordData(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of RecordData");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    // initializePublishers();
    initializeServices();

    initializeFileName();

    start_recording = false;

    spherical_matrix_degree_resolution = 6;
    spherical_matrix_height = 180/spherical_matrix_degree_resolution;
    spherical_matrix_width = 360/spherical_matrix_degree_resolution;
    M = spherical_matrix_height;//180/spherical_matrix_degree_resolution;
    N = spherical_matrix_width;
    
    //initialize variables here, as needed
    //val_to_remember_=0.0; 

    //just initialize with some bullshit values to prevent segmentation fault
    command_translational_velocity[0] = 0;
    command_translational_velocity[1] = 0;
    command_translational_velocity[2] = 0;

    command_angular_velocity[0] = 0;
    command_angular_velocity[1] = 0;
    command_angular_velocity[2] = 0;
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &RecordData::subscriberCallback is a pointer to a member function of RecordData
// "this" keyword is required, to refer to the current instance of RecordData
void RecordData::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

    sphere_matrix_sub = nh_.subscribe("/spherical_matrix", 1, &RecordData::sphericalMatrixCallback, this);
    robot_position_sub = nh_.subscribe("/mavros/local_position/pose", 1, &RecordData::robotPositionCallback, this);
    robot_velocity_sub = nh_.subscribe("/mavros/local_position/velocity", 1, &RecordData::robotVelocityCallback, this);
    robot_local_velocity_sub = nh_.subscribe("/mavros/local_position/odom", 1, &RecordData::robotLocalVelocityCallback, this);

    velocity_command_sub = nh_.subscribe("/mavros/setpoint_velocity/cmd_vel", 1, &RecordData::velocityCommandCallback, this);

    // add more subscribers here, as needed
}


// //member helper function to set up publishers;
// void RecordData::initializePublishers()
// {
//     //ROS_INFO("Initializing Publishers");
//     //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("example_class_output_topic", 1, true); 
//     //add more publishers, as needed
//     // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
// }

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void RecordData::initializeServices()
{
    ROS_INFO("Initializing Services");
    start_recording_service_ = nh_.advertiseService("start_recording",
                                                   &RecordData::startRecordingServiceCallback,
                                                   this);  
    stop_recording_service_ = nh_.advertiseService("stop_recording",
                                                   &RecordData::stopRecordingServiceCallback,
                                                   this); 
    // add more services here, as needed
}

void RecordData::initializeFileName(){

    // to put date and time into the file name:
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%d-%m-%Y_%I:%M:%S",timeinfo);
    std::string date_and_time(buffer);

    filename = "/home/thorstef/catkin_ws/src/obstacle-avoidance/data/" + date_and_time + ".txt";

}

//member function implementation for a service callback function
bool RecordData::startRecordingServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("Start recording service callback activated");
    
    start_recording = true;

    response.success = true; // boring, but valid response info
    response.message = "Starting recording";
    return true;
}

bool RecordData::stopRecordingServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("Stop recording service callback activated");
    
    start_recording = false;

    response.success = true; // boring, but valid response info
    response.message = "Stopping recording";
    return true;
}

void RecordData::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){
    // find smallest distance to obstacle...

    if(start_recording){


        double secs =ros::Time::now().toSec();

        float dstride0 = matrix_msg->layout.dim[0].stride;
        float dstride1 = matrix_msg->layout.dim[1].stride;
        nearest_obstacle_distance = 100;
        //for (m = M*2 +1; m< M*3 ; m++){ // to get rid of some shit values at m=1 ...
        for (m = 1; m< M ; m++){
            for (n = 0; n< N ; n++){
                //sphere_matrix[m][n] = matrix_msg->data[m*dstride1 + n];
                if(matrix_msg->data[m*dstride1 + n] < nearest_obstacle_distance && matrix_msg->data[m*dstride1 + n] > 0.001 ) {
                    nearest_obstacle_distance = matrix_msg->data[m*dstride1 + n];
                    nearest_obstacle_distance_n = n;
                    nearest_obstacle_distance_m = m;
                }
            } 
        }
        //cout << "nearest_obstacle_distance: " << nearest_obstacle_distance << endl; 

        // Maybe write to file here:
        const char *filename_char = filename.c_str();
        

        ofstream myfile;
        myfile.open (filename_char, fstream::app);

        myfile << ToString(secs) << " " << ToString(nearest_obstacle_distance) << " " 
            << ToString(robot_position[0]) << " " << ToString(robot_position[1]) << " "<< ToString(robot_position[2]) << " " 
            << ToString(robot_orientation[0]) << " " << ToString(robot_orientation[1]) << " "<< ToString(robot_orientation[2]) << " "  << ToString(robot_orientation[3]) << " "
            << ToString(robot_translational_velocity[0]) << " " << ToString(robot_translational_velocity[1]) << " " << ToString(robot_translational_velocity[2]) << " "
            << ToString(robot_angular_velocity[0]) << " " << ToString(robot_angular_velocity[1]) << " " << ToString(robot_angular_velocity[2]) << " "
            << ToString(local_robot_translational_velocity[0]) << " " << ToString(local_robot_translational_velocity[1]) << " " << ToString(local_robot_translational_velocity[2]) << " "
            << ToString(local_robot_angular_velocity[0]) << " " << ToString(local_robot_angular_velocity[1]) << " " << ToString(local_robot_angular_velocity[2]) << " "
            << ToString(command_translational_velocity[0]) << " " << command_translational_velocity[1] << " " << command_translational_velocity[1] << " "
            << ToString(command_angular_velocity[0]) << " " << ToString(command_angular_velocity[1]) << " " << ToString(command_angular_velocity[2]) << endl;

        myfile.close();
    }
}


void RecordData::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input) {

    robot_position[0] = input->pose.position.x;
    robot_position[1] = input->pose.position.y;
    robot_position[2] = input->pose.position.z;

    robot_orientation[0] = input->pose.orientation.x;
    robot_orientation[1] = input->pose.orientation.y;
    robot_orientation[2] = input->pose.orientation.z;
    robot_orientation[3] = input->pose.orientation.w;
}

void RecordData::robotVelocityCallback(const geometry_msgs::TwistStampedConstPtr& input) {

    robot_translational_velocity[0] = input->twist.linear.x;
    robot_translational_velocity[1] = input->twist.linear.y;
    robot_translational_velocity[2] = input->twist.linear.z;

    robot_angular_velocity[0] = input->twist.angular.x;
    robot_angular_velocity[1] = input->twist.angular.y;
    robot_angular_velocity[2] = input->twist.angular.z;
}


void RecordData::velocityCommandCallback(const geometry_msgs::TwistStampedConstPtr& input) {

    command_translational_velocity[0] = input->twist.linear.x;
    command_translational_velocity[1] = input->twist.linear.y;
    command_translational_velocity[2] = input->twist.linear.z;

    command_angular_velocity[0] = input->twist.angular.x;
    command_angular_velocity[1] = input->twist.angular.y;
    command_angular_velocity[2] = input->twist.angular.z;
}

void RecordData::robotLocalVelocityCallback(const nav_msgs::OdometryConstPtr& input) {

    local_robot_translational_velocity[0] = input->twist.twist.linear.x;
    local_robot_translational_velocity[1] = input->twist.twist.linear.y;
    local_robot_translational_velocity[2] = input->twist.twist.linear.z;

    local_robot_angular_velocity[0] = input->twist.twist.angular.x;
    local_robot_angular_velocity[1] = input->twist.twist.angular.y;
    local_robot_angular_velocity[2] = input->twist.twist.angular.z;
}


template <typename T>
string RecordData::ToString(T val)
{
  stringstream stream;
  stream << val;
  return stream.str();
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "RecordData"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type RecordData");
    RecordData recordData(&nh);  //instantiate an RecordData object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 