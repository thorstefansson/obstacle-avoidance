// this header incorporates all the necessary #include files and defines the class "SonarsToPointCloud"
#include "sonars_to_point_cloud.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
SonarsToPointCloud::SonarsToPointCloud(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of SonarsToPointCloud");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    
    //initialize variables here, as needed
    //sonar_up_offset = {-0.05, 0.0, 0.13};
    //sonar_down_offset = {-0.05, 0.0, -0.05};
    sonar_up_offset[0]=-0.05;
    sonar_up_offset[1]=0.0;
    sonar_up_offset[2]=0.13;
    sonar_down_offset[0]= -0.05;
    sonar_down_offset[1]=0.0;
    sonar_down_offset[2] = -0.05;
    resolution = 0.05;
    pi = 3.14159265359;
    sonar_half_angle_width =27.5*pi/180;

    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &SonarsToPointCloud::subscriberCallback is a pointer to a member function of SonarsToPointCloud
// "this" keyword is required, to refer to the current instance of SonarsToPointCloud
void SonarsToPointCloud::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &SonarsToPointCloud::subscriberCallback,this);

    sonar_up_subscriber_ = nh_.subscribe("/sonar_up/range", 1, &SonarsToPointCloud::sonarupCallback, this);    //<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
    sonar_down_subscriber_ = nh_.subscribe("/sonar_down/range", 1, &SonarsToPointCloud::sonardownCallback, this);  
    // add more subscribers here, as needed
}


//member helper function to set up publishers;
void SonarsToPointCloud::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pub_sonar_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_to_octomap", 1, true);
    //pub_sonar_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/sonar_point_cloud", 1, true);
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("example_class_output_topic", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}


void SonarsToPointCloud::sonarupCallback(const sensor_msgs::RangeConstPtr& input) {

    range_up = input->range;
    //cout << "range up:" <<range_up << endl;

    // Now make point cloud from the sonar data..

    //-------------------Transform to point cloud:------------------------

    double r, rho, phi, theta, dphi;

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "base_link";
    msg->height = 1;
    
    int count = 0;

    dphi = asin(resolution/range_up);
    for(double phi = 0; phi < sonar_half_angle_width; phi+=dphi){
        for(double theta = 0; theta<2*pi; theta += asin(resolution/(range_up*sin(phi)))){
            rho = range_up;
            r = rho * sin(phi);             
            x = r * cos(theta) + sonar_up_offset[0];
            y = r * sin(theta) + sonar_up_offset[1];
            z = rho * cos(phi) + sonar_up_offset[2];
            msg->points.push_back (pcl::PointXYZ(x,y,z));
            count ++;
        }
    }


    //now for range down:

    dphi = asin(resolution/range_down);
    for(double phi = 0; phi < sonar_half_angle_width; phi+=dphi){
        for(double theta = 0; theta<2*pi; theta += asin(resolution/(range_down*sin(phi)))){
            rho = -range_down;  // Minus because we are looking down!!!
            r = rho * sin(phi);
            //r = rho * cos(phi);             
            x2 = r * cos(theta) + sonar_down_offset[0];
            y2 = r * sin(theta) + sonar_down_offset[1];
            z2 = rho * cos(phi) + sonar_down_offset[2];
            //z = rho * sin(phi) + sonar_up_offset[2];
            msg->points.push_back (pcl::PointXYZ(x2,y2,z2));
            count ++;
        }
    }

    //cout << "count: " << count << endl;

    msg->width = count;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*msg, output);
    // Publish the data
    pub_sonar_cloud.publish (output);

}

void SonarsToPointCloud::sonardownCallback(const sensor_msgs::RangeConstPtr& input) {
    range_down = input->range;
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "sonarsToPointCloud"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type SonarsToPointCloud");
    SonarsToPointCloud sonarsToPointCloud(&nh);  //instantiate an SonarsToPointCloud object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 