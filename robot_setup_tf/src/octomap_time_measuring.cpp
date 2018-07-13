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


// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "octomap_time_measuring.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
OctomapTimeMeasuring::OctomapTimeMeasuring(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of OctomapTimeMeasuring");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    // initializePublishers();
    // initializeServices();

    previous_time_of_octomap_callback = 0;

    initializeFileName();
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void OctomapTimeMeasuring::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);
    octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &OctomapTimeMeasuring::octoMapCallback, this);
    // add more subscribers here, as needed
}

//member helper function to set up services:
// // similar syntax to subscriber, required for setting up services outside of "main()"
// void OctomapTimeMeasuring::initializeServices()
// {
//     ROS_INFO("Initializing Services");
//    // minimal_service_ = nh_.advertiseService("example_minimal_service",&ExampleRosClass::serviceCallback,this);  
//     // add more services here, as needed

//     ///octomap_server/clear_bbx
//     bbx_service_client = nh_.serviceClient<octomap_msgs::BoundingBoxQuery>("octomap_server/clear_bbx");
// }

// //member helper function to set up publishers;
// void OctomapTimeMeasuring::initializePublishers()
// {
//     ROS_INFO("Initializing Publishers");

//     //add more publishers, as needed
//     // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
// }

void OctomapTimeMeasuring::initializeFileName(){

    // to put date and time into the file name:
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%d-%m-%Y_%I:%M:%S",timeinfo);
    std::string date_and_time(buffer);

    filename_octomap = "/home/thorstef/catkin_ws/src/obstacle-avoidance/data/octomap_callback_runtime/" + date_and_time + ".txt";
}


void OctomapTimeMeasuring::octoMapCallback(const octomap_msgs::OctomapConstPtr& octomap_msg){

    time_between_octomap_callbacks =ros::Time::now().toSec() - previous_time_of_octomap_callback;
    previous_time_of_octomap_callback = ros::Time::now().toSec();

    double secs =ros::Time::now().toSec();

    const char *filename_char = filename_octomap.c_str();

    ofstream myfile;
    myfile.open (filename_char, fstream::app);

    myfile << ToString(secs) << " " << ToString(time_between_octomap_callbacks) << endl;

    myfile.close();
}





template <typename T>
string OctomapTimeMeasuring::ToString(T val)
{
  stringstream stream;
  stream << val;
  return stream.str();
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "OctomapTimeMeasuring"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type OctomapTimeMeasuring");
    OctomapTimeMeasuring OctomapTimeMeasuring(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 