// this header incorporates all the necessary #include files and defines the class "TeraRangerToPointCloud"
#include "teraranger_to_point_cloud.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
TeraRangerToPointCloud::TeraRangerToPointCloud(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of TeraRangerToPointCloud");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    
    //initialize variables here, as needed
    //sonar_up_offset = {-0.05, 0.0, 0.13};
    //sonar_down_offset = {-0.05, 0.0, -0.05};

    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &TeraRangerToPointCloud::subscriberCallback is a pointer to a member function of TeraRangerToPointCloud
// "this" keyword is required, to refer to the current instance of TeraRangerToPointCloud
void TeraRangerToPointCloud::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &TeraRangerToPointCloud::subscriberCallback,this);

    // sonar_up_subscriber_ = nh_.subscribe("/sonar_up/range", 1, &TeraRangerToPointCloud::sonarupCallback, this);    //<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
    // sonar_down_subscriber_ = nh_.subscribe("/sonar_down/range", 1, &TeraRangerToPointCloud::sonardownCallback, this);  

    teraranger_subscriber_ = nh_.subscribe("/teraranger/range", 1, &TeraRangerToPointCloud::teraRangerCallback, this);  
    
    // add more subscribers here, as needed
}


//member helper function to set up publishers;
void TeraRangerToPointCloud::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pub_teraranger_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_to_octomap", 1, true); 
}


void TeraRangerToPointCloud::teraRangerCallback(const sensor_msgs::RangeConstPtr& input) {
    double range = input->range;


    if(range < 14){ /// because its returnin 14 when it should be returning 0...
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "teraranger";
        msg->height = 1;

        msg->width = 1;

        msg->points.push_back (pcl::PointXYZ(range, 0,0));

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*msg, output);
        // Publish the data
        pub_teraranger_cloud.publish (output);    
    }
    
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "TeraRangerToPointCloud"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type TeraRangerToPointCloud");
    TeraRangerToPointCloud TeraRangerToPointCloud(&nh);  //instantiate an TeraRangerToPointCloud object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 