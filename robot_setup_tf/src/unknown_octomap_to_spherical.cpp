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
#include "octomap_to_spherical.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
OctomapToSpherical::OctomapToSpherical(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of OctomapToSpherical");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    //val_to_remember_=0.0; 
    resolution = 0.1;
    //resolution = 0.1;
    pi = 3.14159265359;

    box_width_m = 10.0;
    //box_width_m = 5.0;

    //robot_radius = 0.35;//0.4;
    robot_radius = 0.3;
    safety_distance = 0.4;

    laser_vertical_offset = 0.13;

    
    // Height and width of our spherical occupancy matrix:
    // for 3 degrees resolution: M = 60;N = 120;

    deg_resolution = 6;
    M = 180 / deg_resolution;
    N = 360 / deg_resolution;

    half_box_width_m = box_width_m / 2;

    half_box_width_m_sq = pow(half_box_width_m, 2);
    // can also do tests/waits to make sure all required services, topics, etc are alive

    initializeOctomap();
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void OctomapToSpherical::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);

    robot_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &OctomapToSpherical::robotPositionCallback, this);  
    octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &OctomapToSpherical::octoMapCallback, this);
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void OctomapToSpherical::initializeServices()
{
    ROS_INFO("Initializing Services");
   // minimal_service_ = nh_.advertiseService("example_minimal_service",&ExampleRosClass::serviceCallback,this);  
    // add more services here, as needed

    ///octomap_server/clear_bbx
    bbx_service_client = nh_.serviceClient<octomap_msgs::BoundingBoxQuery>("octomap_server/clear_bbx");
}

//member helper function to set up publishers;
void OctomapToSpherical::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("example_class_output_topic", 1, true); 

    pub = nh_.advertise<sensor_msgs::PointCloud2>("/spherical_cloud", 1, true);
    pub_matrix = nh_.advertise<std_msgs::Float32MultiArray>("/spherical_matrix", 1, true);
    //pub_subgoal_matrix = nh_.advertise<std_msgs::Float32MultiArray>("/subgoal_matrix", 1, true);




    //pub_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/bounding_box", 1, true);
    pub_init_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_to_octomap", 1, true);
    //pub_subgoal_cloud= nh_.advertise<sensor_msgs::PointCloud2>("subgoal_cloud", 1, true);

    //pub_points_checked_cloud= nh_.advertise<sensor_msgs::PointCloud2>("points_checked_cloud", 1, true);


    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

void OctomapToSpherical::initializeOctomap()
{
    // initialize the octomap in here..

    // First create point cloud that we publish to the topic that changes octomap,
    //to make a free box or sphere around the initial robot position

    double width = 20*resolution; // 20 is for 1m^3 box around robot when octomap resolution is 5 cm
    double robot_height_from_ground = 0.15+laser_vertical_offset; // its actually between 0.05 - 0.1 according to topic but whatever.. 

    PointCloud::Ptr cloud_msg (new PointCloud);
    cloud_msg->header.frame_id = "laser_link";
    cloud_msg->height = 1;
    // change this:
    //cloud_msg->width = ceil(width/(resolution*resolution) * 6);

    cout << "cloud size: " <<  width/(resolution*resolution) * 6;

    int count = 0;
    // Fill in sides where z constant:
    for(float x = -width/2 ; x < width/2 ; x += resolution){
        for (float y = -width/2; y < width/2; y += resolution)
        {
            cloud_msg->points.push_back (pcl::PointXYZ(x,y,-robot_height_from_ground));  // ? how high is base_link above ground??
            cloud_msg->points.push_back (pcl::PointXYZ(x,y,width-robot_height_from_ground));
            count+=2;
        }
    }
    // Fill in side where x constant
    for(float y = -width/2 ; y < width/2 ; y += resolution){
        for (float z = -robot_height_from_ground; z < width - robot_height_from_ground; z += resolution)
        {
            cloud_msg->points.push_back (pcl::PointXYZ(-width/2,y,z));  // ? how high is base_link above ground??
            cloud_msg->points.push_back (pcl::PointXYZ(width/2,y,z));
            count+=2;
        }
    }
    // Fill in side where y constant
    for(float x = -width/2 ; x < width/2 ; x += resolution){
        for (float z = -robot_height_from_ground; z < width - robot_height_from_ground; z += resolution)
        {
            count+=2;
            cloud_msg->points.push_back (pcl::PointXYZ(x,-width/2,z));  // ? how high is base_link above ground??
            cloud_msg->points.push_back (pcl::PointXYZ(x,width/2,z));
        }
    }

    //cout << " count: " << count <<  endl; 
    //cout << "another count: " << cloud_msg->points.size() << endl;
    //cout << "multiple: " <<     cloud_msg->height * cloud_msg->width <<  endl;

    bool assertion = (cloud_msg->height * cloud_msg->width == cloud_msg->points.size());
    cloud_msg->width = count;
    //cout << "assertion: " << assertion << endl;
    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*cloud_msg, cloud_output);
    // Publish the data
    pub_init_cloud.publish (cloud_output);

    ros::Duration(0.5).sleep();
    count = 0;
    ros::Rate r(10); // 10 hz
    while (count < 50)
    {
        pub_init_cloud.publish (cloud_output);
        count++;
        r.sleep();
    }

    // Then call the /octomap_server/clear_bbx service, to clear out the boundary of the box / sphere 

    octomap_msgs::BoundingBoxQuery srv;

    geometry_msgs::Point min;
    min.x = -width; min.y = -width; min.z = 0;
    cout << min << endl;
    geometry_msgs::Point max; 
    max.x = width; max.y = width; max.z = 1.5 * width;
    cout << max << endl;

    srv.request.min = min;
    srv.request.max = max;

    if(bbx_service_client.call(srv)){
        cout << "bbx service called" << endl;
    }
    else{
        cout << "bbx service NOT called" << endl;
    }


    // this is sort of a lame solution, but should hopefully work...

    // if we use the tree->search method to build the spherical occupancy matrix, 
    // its good to have a model of the sphere within the 3d box matrix:

    //int box_width = box_width_m/resolution;
    //cout << "box_width = " << box_width << endl;
    //int sphere_matrix [box_width] [box_width] [box_width] = {0};

    int half_matrix_width = half_box_width_m/resolution-1;

    //cout << "half_box_width_m: " << half_box_width_m << "resolution: " << resolution << "half_matrix_width: " << half_matrix_width << endl;

    int min_val;

    /*if ((int)(box_width_m/resolution) % 2 == 0){
        // even number
        min_val = 1;
    }
    else min_val = 0;*/
    min_val = 0;

    double ivalue, jvalue, kvalue;
    for (int i = min_val ; i <= half_matrix_width ; i ++){
        for (int j = i ; j <= half_matrix_width ; j ++){
            for (int k = j ; k <= half_matrix_width ; k ++){
                ivalue = i*resolution + resolution/2;
                jvalue = j*resolution + resolution/2;
                kvalue = k*resolution + resolution/2;
                //if(ivalue + jvalue + kvalue < half_box_width_m){
                if(pow(ivalue,2) + pow(jvalue,2) + pow(kvalue,2) < half_box_width_m_sq){
                    // it is within sphere
                    sphere_model[i+ half_matrix_width][j+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[i+ half_matrix_width][k+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][i+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][k+ half_matrix_width][i+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][i+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][j+ half_matrix_width][i+ half_matrix_width] = true;

                    sphere_model[-i+ half_matrix_width][j+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[-i+ half_matrix_width][k+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][-i+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][k+ half_matrix_width][-i+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][-i+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][j+ half_matrix_width][-i+ half_matrix_width] = true;

                    sphere_model[i+ half_matrix_width][-j+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[i+ half_matrix_width][k+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][i+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][k+ half_matrix_width][i+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][i+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][-j+ half_matrix_width][i+ half_matrix_width] = true;

                    sphere_model[i+ half_matrix_width][j+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[i+ half_matrix_width][-k+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][i+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][-k+ half_matrix_width][i+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][i+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][j+ half_matrix_width][i+ half_matrix_width] = true;

                    sphere_model[-i+ half_matrix_width][-j+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[-i+ half_matrix_width][k+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][-i+ half_matrix_width][k+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][k+ half_matrix_width][-i+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][-i+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[k+ half_matrix_width][-j+ half_matrix_width][-i+ half_matrix_width] = true;

                    sphere_model[-i+ half_matrix_width][-j+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[-i+ half_matrix_width][-k+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][-i+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][-k+ half_matrix_width][-i+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][-i+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][-j+ half_matrix_width][-i+ half_matrix_width] = true;

                    sphere_model[i+ half_matrix_width][-j+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[i+ half_matrix_width][-k+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][i+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[-j+ half_matrix_width][-k+ half_matrix_width][i+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][i+ half_matrix_width][-j+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][-j+ half_matrix_width][i+ half_matrix_width] = true;  

                    sphere_model[-i+ half_matrix_width][j+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[-i+ half_matrix_width][-k+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][-i+ half_matrix_width][-k+ half_matrix_width] = true;
                    sphere_model[j+ half_matrix_width][-k+ half_matrix_width][-i+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][-i+ half_matrix_width][j+ half_matrix_width] = true;
                    sphere_model[-k+ half_matrix_width][j+ half_matrix_width][-i+ half_matrix_width] = true;  
                }
                else{
                    //not within sphere
                    sphere_model[i+ half_matrix_width][j+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[i+ half_matrix_width][k+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][i+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][k+ half_matrix_width][i+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][i+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][j+ half_matrix_width][i+ half_matrix_width] = false;

                    sphere_model[-i+ half_matrix_width][j+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[-i+ half_matrix_width][k+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][-i+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][k+ half_matrix_width][-i+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][-i+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][j+ half_matrix_width][-i+ half_matrix_width] = false;

                    sphere_model[i+ half_matrix_width][-j+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[i+ half_matrix_width][k+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][i+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][k+ half_matrix_width][i+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][i+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][-j+ half_matrix_width][i+ half_matrix_width] = false;

                    sphere_model[i+ half_matrix_width][j+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[i+ half_matrix_width][-k+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][i+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][-k+ half_matrix_width][i+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][i+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][j+ half_matrix_width][i+ half_matrix_width] = false;

                    sphere_model[-i+ half_matrix_width][-j+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[-i+ half_matrix_width][k+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][-i+ half_matrix_width][k+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][k+ half_matrix_width][-i+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][-i+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[k+ half_matrix_width][-j+ half_matrix_width][-i+ half_matrix_width] = false;

                    sphere_model[-i+ half_matrix_width][-j+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[-i+ half_matrix_width][-k+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][-i+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][-k+ half_matrix_width][-i+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][-i+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][-j+ half_matrix_width][-i+ half_matrix_width] = false;

                    sphere_model[i+ half_matrix_width][-j+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[i+ half_matrix_width][-k+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][i+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[-j+ half_matrix_width][-k+ half_matrix_width][i+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][i+ half_matrix_width][-j+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][-j+ half_matrix_width][i+ half_matrix_width] = false;

                    sphere_model[-i+ half_matrix_width][j+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[-i+ half_matrix_width][-k+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][-i+ half_matrix_width][-k+ half_matrix_width] = false;
                    sphere_model[j+ half_matrix_width][-k+ half_matrix_width][-i+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][-i+ half_matrix_width][j+ half_matrix_width] = false;
                    sphere_model[-k+ half_matrix_width][j+ half_matrix_width][-i+ half_matrix_width] = false;  

                }

            }
        }
    }

    /*cout << "Sphere model: " << endl;
    // print the sphere model to verify
    for(int i = 0; i <= half_matrix_width; i++){
        for(int j = 0; j< box_width_m/resolution; j++){
            for(int k = 0; k< box_width_m/resolution; k++){
                //if(sphere_model[i][j][k]) cout << "1 ";
                cout << sphere_model[i][j][k] << " " ;
            }
            cout << endl;
        }
        cout << endl;
    }

    cout << "done printing sphere model" << endl;*/

}



/*
// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
void OctomapToSpherical::subscriberCallback(const std_msgs::Float32& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the current value of val_to_remember_ 
}*/


void OctomapToSpherical::octoMapCallback(const octomap_msgs::OctomapConstPtr& octomap_msg){


    //octomap::OcTree* tree = octomap_msgs::binaryMsgToMap(*msg);


    //cout << "in octomap callback (to spherical)" << endl;
    // if using non binary:
    //octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    //octomap::OcTree* tree = dynamic_cast<OcTree*>(msg);

    octomap::AbstractOcTree* oldtree = octomap_msgs::binaryMsgToMap(*octomap_msg);
    octomap::OcTree* tree = (octomap::OcTree*)oldtree;


    int m,n;
    double r, rho, phi, theta;

    //cout << "here" << endl;
    double sphere_matrix [M][N] = {0}, sphere_matrix_with_unknown[M][N] = {0};
    double r_sq, distance_sq, node_size;
    double length_to_check;// = resolution / sin(deg_resolution*pi/180); // This is at horizontal plane... 
    double final_length_to_check;
    int node_size_multiple, size_distance_factor;

    int n_size_distance_factor, m_size_distance_factor;

    double x,y,z;

    // Now we need to rotate the coordinates so they are in robot frame.....
    // would be good to find some other solution to save computational power.. 

    // To get the point in the same frame as the robot we need to turn the it
    // to do this we find the inverse of the robot quaternion and use it to turn the point

    // inverse of robot quaternion:
    Vector3d robot_orientation_inverse_v;
    Vector3d roivxp; // robot_orientation_inverse x point
    float robot_orientation_inverse_w;

    float robot_orientation_sum_squared = pow(robot_orientation[0],2) + pow(robot_orientation[1],2) + pow(robot_orientation[2],2)
    +pow(robot_orientation[3],2);

    robot_orientation_inverse_w = robot_orientation[3] / robot_orientation_sum_squared;

    robot_orientation_inverse_v[0] = -robot_orientation[0] / robot_orientation_sum_squared;
    robot_orientation_inverse_v[1] = -robot_orientation[1] / robot_orientation_sum_squared;
    robot_orientation_inverse_v[2] = -robot_orientation[2] / robot_orientation_sum_squared;
    Vector3d point, point_robot_frame;




    

    //cout << "box_width = " << box_width << endl;
    //int bounding_box_matrix [box_width] [box_width] [box_width] = {0};
    //cout << "Hmm" << endl;

    //cout << "box_width: " << box_width << endl;


    /*PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "base_link";
    msg->height = 1;
    int countpoints = 0;*/


    // We want to publish the position and orientation of the robot at the time of this matrix being calculated to make accurate sub goal distance computations

    // so let's allocate some space in the sphere matrix for that.. 
    sphere_matrix[0][0] = robot_position[0];
    sphere_matrix[0][1] = robot_position[1];
    sphere_matrix[0][2] = robot_position[2];
    sphere_matrix[0][3] = robot_orientation[0];
    sphere_matrix[0][4] = robot_orientation[1];
    sphere_matrix[0][5] = robot_orientation[2];
    sphere_matrix[0][6] = robot_orientation[3];
    
    point3d min;
    min.x() = robot_position[0] - half_box_width_m; min.y() = robot_position[1] - half_box_width_m; min.z() = robot_position[2] - half_box_width_m;
    //cout << "min"<< min << endl;
    point3d max; 
    max.x() = robot_position[0] + half_box_width_m; max.y() = robot_position[1] + half_box_width_m; max.z() = robot_position[2] + half_box_width_m;
    //cout << "max" <<max << endl;

    
    // ------------------------------------------------------ TO LABEL UNKNOWN AS OCCUPIED: -------------------------------------------------------------------------

    
    int box_width = box_width_m/resolution;
    node_size = resolution;
    int countx=0, county=0, countz=0;
    for (double ix = min.x() + resolution/2; ix < max.x(); ix += resolution){
        for (double iy = min.y() + resolution /2; iy < max.y(); iy += resolution){
            for (double iz = min.z()+ resolution /2; iz < max.z(); iz += resolution)
            {
                //cout << "matrix coordinates: " << countx << " " << county << " " countz << " ";
                if(sphere_model[countx][county][countz]){
                    // within sphere..

                    if (!tree->search(ix,iy,iz))
                    {
                        //This cell is unknown, so mark as occupied ... 
                        point[0]=ix - robot_position[0]; point[1]=iy - robot_position[1]; point[2]=iz - robot_position[2];

                        roivxp = cross(robot_orientation_inverse_v, point);
                        point_robot_frame = point + 2*robot_orientation_inverse_w*roivxp + 2*cross(robot_orientation_inverse_v, roivxp);
                        
                        // another way:
                        //point_robot_frame = 2.0f * dot(robot_orientation_inverse_v, point) * robot_orientation_inverse_v
                        //                    + (pow(robot_orientation_inverse_w,2) - dot(robot_orientation_inverse_v, robot_orientation_inverse_v)) * point
                        //                    + 2.0f * robot_orientation_inverse_w * cross(robot_orientation_inverse_v, point);
                        // One more way:

                        //point_robot_frame = q_robot_orientation * tf_point * q_robot_orientation_inverse;

                        x=point_robot_frame[0]; y=point_robot_frame[1]; z=point_robot_frame[2];
                        
                        r_sq = pow(x,2)+pow(y,2);
                        distance_sq = r_sq+pow(z,2);

                        //transform to spherical coordinates:
                        r = sqrt(r_sq);
                        rho = sqrt( distance_sq);  //distance from base link
                        theta = asin(y / r);            //angle from x (forward) axis in xy plane
                        phi = asin(z / rho);            //angle from xy plane    

                        if(x < 0){
                            theta = pi - theta;
                        }

                        // all right so now hopefully theta is between -pi - pi and phi between
                        // - pi/2 - pi/2
                        
                        //find potential placement in matrix
                        m = floor((phi + pi/2) * M / pi);
                        n = floor((theta + pi) * M / pi);       // should look further into this....................
                        //cout << "m:" << m << "n:" << n << endl;

                        if(n>=N){
                            n=n-N;
                        }
                        if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
                            if( rho < sphere_matrix_with_unknown[m][n] || sphere_matrix_with_unknown[m][n] ==0){
                                //cout << "in" << endl;
                                sphere_matrix_with_unknown[m][n] = rho - node_size/2;
                                //sphere_matrix_with_unknown[m][n] = rho - node_size/2;
                            }
                        }
                        else{
                            cout << "m: " << m << " n:" << n << endl;
                        }

                        final_length_to_check = resolution*1.4142/ sin(deg_resolution*pi/180);//length_to_check* abs(sin(fmod(phi, (pi/2)) * 2));  * factor(1 + 0.4142*sin(abs(fmod(phi, (pi/2))) * 2))

                        if(rho*cos(phi) < final_length_to_check  ){
                            //cout << "special case" << endl;
                            // now we need to fill out more than one element in the sphere matrix
                            m_size_distance_factor =  floor( final_length_to_check / rho); //floor(4 * node_size_multiple / ceil(rho*4)) ;
                            
                            n_size_distance_factor = floor( final_length_to_check /(rho*cos(phi)));

                            //if(size_distance_factor >=1){ 

                            for(int i = m - m_size_distance_factor ; i <= m+m_size_distance_factor ; i++){
                                for(int j = n - n_size_distance_factor ; j <= n+n_size_distance_factor ; j++){
                                    if(i> -1 && j> -1 && i < M && j< N){ //?
                                        if( rho < sphere_matrix_with_unknown[i][j] || sphere_matrix_with_unknown[i][j] == 0){
                                            sphere_matrix_with_unknown[i][j] = rho - node_size/2;
                                        }
                                    }
                                }
                            }
                            //}
                        }
                    }
                    else if(tree->search(ix,iy,iz)->getValue() > 0){
                        // this cell is occupied
                        //bounding_box_matrix [countx] [county] [countz] = 1;

                        point[0]=ix - robot_position[0]; point[1]=iy - robot_position[1]; point[2]=iz - robot_position[2];

                        //roivxp = cross(robot_orientation_inverse_v, point);
                        //point_robot_frame = point + 2*robot_orientation_inverse_w*roivxp + 2*cross(robot_orientation_inverse_v, roivxp);
                        // another way:
                        point_robot_frame = 2.0f * dot(robot_orientation_inverse_v, point) * robot_orientation_inverse_v
                                            + (pow(robot_orientation_inverse_w,2) - dot(robot_orientation_inverse_v, robot_orientation_inverse_v)) * point
                                            + 2.0f * robot_orientation_inverse_w * cross(robot_orientation_inverse_v, point);

                        // One more way:


                        x=point_robot_frame[0]; y=point_robot_frame[1]; z=point_robot_frame[2];

                        r_sq = pow(x,2)+pow(y,2);
                        distance_sq = r_sq+pow(z,2);

                        //transform to spherical coordinates:
                        r = sqrt(r_sq);
                        rho = sqrt( distance_sq);  //distance from base link
                        theta = asin(y / r);            //angle from x (forward) axis in xy plane
                        phi = asin(z / rho);            //angle from xy plane    

                        if(x < 0){
                            theta = pi - theta;
                        }

                        // all right so now hopefully theta is between -pi - pi and phi between
                        // - pi/2 - pi/2
                        
                        //find potential placement in matrix
                        m = floor((phi + pi/2) * M / pi);
                        n = floor((theta + pi) * M / pi);       // should look further into this....................
                        //cout << "m:" << m << "n:" << n << endl;

                        if(n>=N){
                            n=n-N;
                        }
                        if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
                            if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] == 0){
                                sphere_matrix[m][n] = rho - node_size/2;
                                //msg->points.push_back (pcl::PointXYZ(x,y,z));
                                //countpoints++;
                            }
                            if( rho < sphere_matrix_with_unknown[m][n] || sphere_matrix_with_unknown[m][n] == 0){
                                sphere_matrix_with_unknown[m][n] = rho - node_size/2;
                            }
                        }
                        else{
                            cout << "m: " << m << " n:" << n << endl;
                        }

                        final_length_to_check = resolution*1.4142/ sin(deg_resolution*pi/180);//length_to_check* abs(sin(fmod(phi, (pi/2)) * 2));  * factor(1 + 0.4142*sin(abs(fmod(phi, (pi/2))) * 2))

                        if(rho*cos(phi) < final_length_to_check  ){
                            //cout << "special case" << endl;
                            // now we need to fill out more than one element in the sphere matrix
                            m_size_distance_factor =  floor( final_length_to_check / rho); //floor(4 * node_size_multiple / ceil(rho*4)) ;
                            
                            n_size_distance_factor = floor( final_length_to_check /(rho*cos(phi)));

                            for(int i = m - m_size_distance_factor ; i <= m+m_size_distance_factor ; i++){
                                for(int j = n - n_size_distance_factor ; j <= n+n_size_distance_factor ; j++){
                                    if(i> -1 && j> -1 && i < M && j< N){ //?
                                        if( rho < sphere_matrix[i][j] || sphere_matrix[i][j] == 0){
                                            sphere_matrix[i][j] = rho - node_size/2;
                                           // msg->points.push_back (pcl::PointXYZ(x,y,z));
                                            //countpoints++;
                                        }
                                        if( rho < sphere_matrix_with_unknown[i][j] || sphere_matrix_with_unknown[i][j] == 0){
                                            sphere_matrix_with_unknown[i][j] = rho - node_size/2;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // else if(tree->search(ix,iy,iz)->getValue() < 0){
                    //     // this cell is free ...
                    //     //bounding_box_matrix [countx] [county] [countz] = -1;
                    // }
                    // else{
                    //     cout << "eitthvad skrytid" << endl;
                    // }


                }
                countz++;
            }
            county++;
            countz =0;
        }
        countx++;
        county = 0;
    }

    sphere_matrix_with_unknown[0][0] = robot_position[0];
    sphere_matrix_with_unknown[0][1] = robot_position[1];
    sphere_matrix_with_unknown[0][2] = robot_position[2];
    sphere_matrix_with_unknown[0][3] = robot_orientation[0];
    sphere_matrix_with_unknown[0][4] = robot_orientation[1];
    sphere_matrix_with_unknown[0][5] = robot_orientation[2];
    sphere_matrix_with_unknown[0][6] = robot_orientation[3];
    

    /*cout << "sphere matrix: " << endl;// << sphere_matrix << endl;
    for (m = M-1; m>=0 ; m--){
        for(n=0;n<N;n++){
            cout << sphere_matrix[m][n] << " " ;
        }
        cout << endl;
    }*/

    /*msg->width = countpoints;
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*msg, output);
    // Publish the data
    pub.publish (output); */

 /*   //-------------------Transform again to point cloud for visualizing in rviz:------------------------

    PointCloud::Ptr cloud_msg (new PointCloud);
    cloud_msg->header.frame_id = "base_link";
    cloud_msg->height = 1;
    cloud_msg->width = 120*120*120;

      //for(int i = 0, max = 120*60; i!=max, i++) {
    for( int x=0; x<box_width; x++){
        for(int y=0; y<box_width; y++){
            for ( int z = 0; z < box_width ; z++){
                if(bounding_box_matrix[x][y][z] >= 0 ){
                    // cell free
                    cloud_msg->points.push_back (pcl::PointXYZ(0,0,0));
                }
                else{
                    // cell (not) occupied
                    cloud_msg->points.push_back (pcl::PointXYZ(x*resolution - half_box_width_m,
                        y*resolution - half_box_width_m,z*resolution - half_box_width_m));
                }
            }
        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*cloud_msg, cloud_output);
    // Publish the data
    pub_cloud.publish (cloud_output);
    */




    // Use:
    //cout << "2" << endl ; // "going into bbx iterator." << endl;

    //double length_to_check = resolution * sqrt(2) / sin(deg_resolution);

    // --------------------------------------------------TO USE BBX ITERATOR, IGNORE UNKNOWN SPACE:-------------------------------------------
/*
    for(OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max),end=tree->end_leafs_bbx(); it!= end; ++it)
    {
        //manipulate node, e.g.:
        //std::cout << "Node center: " << it.getCoordinate() << std::endl;
        //cout << "x: " << it.getX() <<  "y: " << it.getY() << "z: " << it.getZ() << endl;
        //std::cout << "Node size: " << it.getSize() << std::endl;
        //std::cout << "Node value: " << it->getValue() << std::endl;

        // fill in element in bounding box matrix that are affected by node

        if(it->getValue() > 0 ){
            // Node is occupied 
             // check if within sphere:
            //x=it.getX() - robot_position[0]; y=it.getY() - robot_position[1]; z=it.getZ() - robot_position[2];
            point[0]=it.getX() - robot_position[0]; point[1]=it.getY() - robot_position[1]; point[2]=it.getZ() - robot_position[2];

            roivxp = cross(robot_orientation_inverse_v, point);
            point_robot_frame = point + 2*robot_orientation_inverse_w*roivxp + 2*cross(robot_orientation_inverse_v, roivxp);
            
            x=point_robot_frame[0]; y=point_robot_frame[1]; z=point_robot_frame[2];
            
            r_sq = pow(x,2)+pow(y,2);
            distance_sq = r_sq+pow(z,2);
            
            if(distance_sq < half_box_width_m_sq){
                // within sphere
                node_size = it.getSize();
                // find angles and set into sphere matrix:

                //transform to spherical coordinates:
                r = sqrt(r_sq);
                rho = sqrt( distance_sq);  //distance from base link
                theta = asin(y / r);            //angle from x (forward) axis in xy plane
                phi = asin(z / rho);            //angle from xy plane    

                if(x < 0){
                    theta = pi - theta;
                }

                // all right so now hopefully theta is between -pi - pi and phi between
                // - pi/2 - pi/2
                
                //find potential placement in matrix
                m = floor((phi + pi/2) * M / pi);
                n = floor((theta + pi) * M / pi);       // should look further into this....................
                //cout << "m:" << m << "n:" << n << endl;

                if(n>=N){
                    n=n-N;
                }
                //if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
                if(m> 0 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){ // not put anything where m = 0, since we want to put robot position and orientation info there..  
                    if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] ==0){
                        //cout << "in" << endl;
                        sphere_matrix[m][n] = rho - node_size/2;  
                    }
                }
                // else{
                //     cout << "m: " << m << " n:" << n << endl;
                // }

                //since we have boxes of size 3 deg, at 1m distance that is 5 cm, which is the resolution of the octomap...

                // box of size d degrees, at X distance it equals the resolution of octomap and need to fill in more shit.. 
                // so like at: distance * sin(degree_resolution) < octomap_resolution* node_multiple * sqrt(2)  (if at 45 degrees)
                // we're gonna have to fill out more than one element of the occupancy matrix
                // so .. 

                final_length_to_check = node_size/ sin(deg_resolution*pi/180);//length_to_check* abs(sin(fmod(phi, (pi/2)) * 2));  * factor(1 + 0.4142*sin(abs(fmod(phi, (pi/2))) * 2))

                if(rho*cos(phi) < final_length_to_check  ){
                    //cout << "special case" << endl;
                    // now we need to fill out more than one element in the sphere matrix
                    m_size_distance_factor =  floor( final_length_to_check / rho); //floor(4 * node_size_multiple / ceil(rho*4)) ;
                    
                    n_size_distance_factor = floor( final_length_to_check /(rho*cos(phi)));

                    for(int i = m - m_size_distance_factor ; i <= m+m_size_distance_factor ; i++){
                        for(int j = n - n_size_distance_factor ; j <= n+n_size_distance_factor ; j++){
                            //if(i> -1 && j> -1 && i < M && j< N){ //?
                            if(i> 0 && j> -1 && i < M && j< N){ // not put anything where m = 0, since we want to put robot position and orientation info there..  
                                if( rho < sphere_matrix[i][j] || abs(sphere_matrix[i][j]) < 0.0001){
                                    sphere_matrix[i][j] = rho - node_size/2;
                                   // msg->points.push_back (pcl::PointXYZ(x,y,z));
                                    //countpoints++;
                                }
                            }
                        }
                    }
                }

                
                // node_size_multiple = round(node_size/resolution);
                // if(rho<length_to_check || node_size_multiple >1 ){
                //     //cout << "special case" << endl;
                //     // now we need to fill out more than one element in the sphere matrix
                //     size_distance_factor =  floor(resolution*node_size_multiple*1.4142/(rho * sin(deg_resolution))); //floor(4 * node_size_multiple / ceil(rho*4)) ;
                    
                //     if(size_distance_factor >=1){ // size_distance_factor >=1
                        
                //         // find which quadrant of angle box the center is..
                //         //bool horizontal = signbit((phi + pi/2) * M / pi round((phi + pi/2) * M / pi));
                //         // ah, fuck that, lets just make this simple... and perhaps fill out more than we should.. 

                //         for(int i = m - size_distance_factor ; i <= m+size_distance_factor ; i++){
                //             for(int j = n - size_distance_factor ; j <= n+size_distance_factor ; j++){
                //                 if(i> -1 && j> -1 && i < M && j< N){ //?
                //                     if( rho < sphere_matrix[i][j] || sphere_matrix[i][j] == 0){
                //                             sphere_matrix[i][j] = rho - node_size/2;
                //                     }
                //                 }
                //             }
                //         }
                //     }
                // }
            }
        }

       //if (it.getSize() > 0.05 ) cout << "larger" ;
    }
*/



        //---------------------------------Publish matrix to topic:-----------------------------------------

    /*std_msgs::Float32MultiArray dat;
    int H = M;
    int W = N;
     // fill out message:
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "height";
    dat.layout.dim[1].label = "width";
    dat.layout.dim[0].size = H;
    dat.layout.dim[1].size = W;
    dat.layout.dim[0].stride = H*W;
    dat.layout.dim[1].stride = W;
    dat.layout.data_offset = 0;
    std::vector<float> vec(W*H, 0);
    for (int i=0; i<H; i++)
        for (int j=0; j<W; j++)
            vec[i*W + j] = sphere_matrix[i][j];
    dat.data = vec;

    pub_matrix.publish(dat);*/


 
    //-------------------Transform again to point cloud for visualizing in rviz:------------------------


    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "base_link";
    msg->height = 1;
    msg->width = (M-1)*N;

      //for(int i = 0, max = 120*60; i!=max, i++) {
    for( m=1; m < M; m++){
        for(n=0; n<N; n++){
            if(sphere_matrix[m][n] == 0){
                msg->points.push_back (pcl::PointXYZ(0,0,0));
            }
            else
            {
                //r = rho * sin(phi);
                rho = sphere_matrix[m][n];
                theta = (n+0.5) *  pi / M - pi;
                phi = (m+0.5) * pi / M - pi/2;

                r = rho * cos(phi);
                
                x = r * cos(theta);
                y = r * sin(theta);
                z = rho * sin(phi);
                msg->points.push_back (pcl::PointXYZ(x,y,z));
                //cout << "x: " << x << endl;
            }
        }
    }
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*msg, output);
    // Publish the data
    pub.publish (output); 


    // -------------Try locating sub goals here in order to remove sub goals created on surfaces due to discretization:---------------

    double difference;
    double subgoal_matrix [M][N] = {0};

    Vector3d robot_orientation_v(robot_orientation[0], robot_orientation[1], robot_orientation[2]);
    float robot_orientation_w = robot_orientation[3];


    // --------------To locate sub goals exactly on the boundary: -----------------
/*
    //max is like 27-41 for M = 60
    int m_min = M * 15/60;
    int m_max = M * 45/60;

    //cout << "m min " << m_min << " m max " << m_max << " M " << M << endl;
    // or we could omit sub goals where there is unknown in the octomap.. 
    for( m = m_min ; m < m_max ; m++){  //max is like 27-41 for M = 60
        for( n = 0; n< N-1 ; n++){ //and 50 - 68 for N = 120

            //To make sure we're not on the edge of view of the camera:
            //if(sphere_matrix[m][n] != 0){//robot_radius + safety_distance){

            //Check horizontal difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m][n+1]);
            //if(difference > 2*robot_radius ){// - (robot_radius + safety_distance)) >0.001){ 
            // this means that there is enough difference between these points to count
            // as subgoal points...

            /*cout << "camera_x_offset + max_camera_range : " << camera_x_offset + max_camera_range <<  endl;
            cout << "sphere_matrix[m][n]: " << sphere_matrix[m][n] <<  endl;
            statement = sphere_matrix[m][n] - (camera_x_offset + max_camera_range);
            cout << "statement: " << statement <<  endl;*/
/*
            if(sphere_matrix[m][n] ==0){  //just to get rid of some dumb ass floating point error
                //this means we are at the edge of obstacle

                subgoal_matrix[m][n] = sphere_matrix[m][n+1] + difference / 2;  
   
            }
            else if(sphere_matrix[m][n+1] ==0){
                //this means we are at the edge of obstacle

                subgoal_matrix[m][n+1] = sphere_matrix[m][n] + difference / 2;  
                    
            } 
            else{
                //otherwise we're between obstacles
                if(sphere_matrix[m][n]<sphere_matrix[m][n+1]){
                    subgoal_matrix[m][n+1] = sphere_matrix[m][n] + difference/2;
                }
                else{
                    subgoal_matrix[m][n] = sphere_matrix[m][n+1] + difference/2;
                }
            } 
            //}

            //Check vertical difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m+1][n]);
            //if(difference > 2*robot_radius){
            // this means that there is enough difference between these points to count
            // as subgoal points...

            if(sphere_matrix[m][n] ==0){
                //this means we are at the edge of obstacle
                    subgoal_matrix[m][n] = sphere_matrix[m+1][n] + difference / 2;         
            }
            else if(sphere_matrix[m+1][n] ==0){
                //this means we are at the edge of obstacle
                subgoal_matrix[m+1][n] = sphere_matrix[m][n] + difference / 2;      
            } 
            else{
                //otherwise we're between obstacles
                if(sphere_matrix[m][n]<sphere_matrix[m+1][n]){
                    subgoal_matrix[m+1][n] = sphere_matrix[m][n] + difference/2;
                }
                else{
                    subgoal_matrix[m][n] = sphere_matrix[m+1][n] + difference/2;
                }
            } 
            //}
            //}
        }
    }
    */

        // --------------To locate sub goals a little from the boundary: -----------------


    /*PointCloud::Ptr points_checked_msg (new PointCloud);
    points_checked_msg->header.frame_id = "/world";
    points_checked_msg->height = 1;
*/

    //max is like 27-41 for M = 60
    /*int m_min = 1;//M * 20/60;
    int m_max = M-1; //M * 40/60;

    //cout << "m min " << m_min << " m max " << m_max << " M " << M << endl;
    // or we could omit sub goals where there is unknown in the octomap.. 
    for( m = m_min ; m < m_max ; m++){  //max is like 27-41 for M = 60
        for( n = 1; n< N-2 ; n++){ //and 50 - 68 for N = 120

            //To make sure we're not on the edge of view of the camera:
            //if(sphere_matrix[m][n] != 0){//robot_radius + safety_distance){

            //Check horizontal difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m][n+1]);
            if(difference > 0){// - (robot_radius + safety_distance)) >0.001){ 
            //sphere_matrix[m][n+1] != robot_radius + safety_distance){


    
                if(sphere_matrix[m][n] < 0.0001){  //just to get rid of some dumb ass floating point error
                    //this means we are at the edge of obstacle
                    // and sphere_matrix[m][n] in back
                    
                    subgoal_matrix[m][n-1] = sphere_matrix[m][n+1] + robot_radius*2 + safety_distance;
       
                }
                else if(sphere_matrix[m][n+1] < 0.0001){
                    //this means we are at the edge of obstacle
                    subgoal_matrix[m][n+2] = sphere_matrix[m][n] + robot_radius*2 + safety_distance;    
    
                } 
                else if(difference > 2*robot_radius){
                    // this means that there is enough difference between these points to count
                    // as subgoal points...
                    //otherwise we're between obstacles
                    // Check here if point is located on surface .. 
                    phi = m * pi / M - pi/2;
                    theta = n *  pi / M - pi + deg_resolution/2 * pi/180;

                    if(sphere_matrix[m][n]<sphere_matrix[m][n+1]){
                        
                        rho = sphere_matrix[m][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m][n+2] = rho-resolution/2;
                        else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m][n+2] = rho-resolution/2;

                    }
                    else{

                        rho = sphere_matrix[m][n+1] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m][n-1] = rho-resolution/2;
                        else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m][n-1] = rho-resolution/2;

                    }
                } 
            }

            //Check vertical difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m+1][n]);
            if(difference > 0){
                // this means that there is enough difference between these points to count
                // as subgoal points...



                if(sphere_matrix[m][n] < 0.001){
                    //this means we are at the edge of obstacle
                    subgoal_matrix[m-1][n] = sphere_matrix[m+1][n] + robot_radius*2 + safety_distance;  
                }
                else if(sphere_matrix[m+1][n] < 0.001){
                    //this means we are at the edge of obstacle
                    subgoal_matrix[m+2][n] = sphere_matrix[m][n] + robot_radius*2 + safety_distance;     
                } 
                else if(difference > 2*robot_radius){
                    //otherwise we're between obstacles

                    // Check here if point is located on surface ..
                    phi = m * pi / M - pi/2 + deg_resolution/2 * pi/180;
                    theta = n *  pi / M - pi;


                    if(sphere_matrix[m][n]<sphere_matrix[m+1][n]){

                        rho = sphere_matrix[m][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m+2][n] = rho-resolution/2;
                        else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m+2][n] = rho-resolution/2;
                    }
                    else{

                        rho = sphere_matrix[m+1][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m-1][n] = rho-resolution/2;
                        else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m-1][n] = rho-resolution/2;
                    }
                } 
            }
            //}
        }
    }*/


// --------------To locate sub goals exactly as much from the boundary as needed: -----------------

    /*PointCloud::Ptr points_checked_msg (new PointCloud);
    points_checked_msg->header.frame_id = "/world";
    points_checked_msg->height = 1;
*/

    //max is like 27-41 for M = 60
    int m_min = 1;//M * 20/60;
    int m_max = M-1; //M * 40/60;

    double angular_offset_factor = resolution/sin(deg_resolution*pi/180); // consider using Cspace resolution instead of octomap resolution...
    //cout << "angular_offset_factor : " << angular_offset_factor << endl;
    int angular_offset;

    //cout << "m min " << m_min << " m max " << m_max << " M " << M << endl;
    // or we could omit sub goals where there is unknown in the octomap.. 
    for( m = m_min ; m < m_max ; m++){  //max is like 27-41 for M = 60
        for( n = 0; n< N-1 ; n++){ //and 50 - 68 for N = 120

            //To make sure we're not on the edge of view of the camera:
            //if(sphere_matrix[m][n] != 0){//robot_radius + safety_distance){

            //Check horizontal difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m][n+1]);
            if(difference > 0){// - (robot_radius + safety_distance)) >0.001){ 
            //sphere_matrix[m][n+1] != robot_radius + safety_distance){


                phi = (m+0.5) * pi / M - pi/2;

                if(sphere_matrix[m][n] < 0.0001){  //just to get rid of some dumb ass floating point error
                    //this means we are at the edge of obstacle
                    // and sphere_matrix[m][n] in back
                    angular_offset = ceil(angular_offset_factor / (sphere_matrix[m][n+1]*cos(phi)));
                    if(n+1-angular_offset > -1){
                        //subgoal_matrix[m][n+1-angular_offset] = sphere_matrix[m][n+1] + robot_radius*2;// + safety_distance;  ORIGINAL
                        
                        // to only locate sub goals where we have free space in octomap:
                        theta = (n+1-angular_offset+0.5) * pi / M - pi + deg_resolution/2 * pi/180; // angle at n
                        rho = sphere_matrix[m][n+1] + robot_radius*2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];
                        //if(tree->search(point[0], point[1],point[2])){
                        if(tree->search(point[0], point[1],point[2])){  
                            if(tree->search(point[0], point[1],point[2])->getValue() < 0){ 
                                subgoal_matrix[m][n+1-angular_offset] = sphere_matrix[m][n+1] + robot_radius*2;// + safety_distance;
                            }
                        }
                    } 
                }
                else if(sphere_matrix[m][n+1] < 0.0001){
                    //this means we are at the edge of obstacle

                    angular_offset = ceil(angular_offset_factor / (sphere_matrix[m][n]*cos(phi))); 
                    if(n+angular_offset < N){
                        //subgoal_matrix[m][n+angular_offset] = sphere_matrix[m][n] + robot_radius*2; // + safety_distance;

                        // to only locate sub goals where we have free space in octomap:
                        theta = (n+angular_offset+1.5) * pi / M - pi + deg_resolution/2 * pi/180; // angle at n+1 ...
                        rho = sphere_matrix[m][n] + robot_radius*2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        if(tree->search(point[0], point[1],point[2])){
                            if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m][n+angular_offset] = sphere_matrix[m][n] + robot_radius*2;
                        }
                    }
                } 
                else if(difference > 2*robot_radius){
                    // this means that there is enough difference between these points to count
                    // as subgoal points...
                    //otherwise we're between obstacles
                    // Check here if point is located on surface .. 
                    
                    theta = (n+0.5) *  pi / M - pi + deg_resolution/2 * pi/180;

                    if(sphere_matrix[m][n]<sphere_matrix[m][n+1]){
                        
                        rho = sphere_matrix[m][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        angular_offset = ceil(angular_offset_factor / (sphere_matrix[m][n]*cos(phi)) );

                        if(n+angular_offset < N){
                            // if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m][n+angular_offset] = rho-resolution/2;
                            // else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m][n+angular_offset] = rho-resolution/2;
                            if(tree->search(point[0], point[1],point[2])){
                                if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m][n+angular_offset] = rho-resolution/2;
                            }
                        }

                    }
                    else{

                        rho = sphere_matrix[m][n+1] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));

                        angular_offset = ceil(angular_offset_factor / (sphere_matrix[m][n+1]*cos(phi)) );

                        if(n+1-angular_offset > -1){
                            // if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m][n+1-angular_offset] = rho-resolution/2;
                            // else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m][n+1-angular_offset] = rho-resolution/2; 
                            if(tree->search(point[0], point[1],point[2])){
                                if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m][n+1-angular_offset] = rho-resolution/2; 
                            }   
                        }
                    }
                } 
                //if(angular_offset < 1) cout << "angular offset is: " << angular_offset << " in n difference" << endl;
                //cout << "angular_offset for n: " << angular_offset << endl;
            }

            //Check vertical difference:
            difference = abs(sphere_matrix[m][n]-sphere_matrix[m+1][n]);
            if(difference > 0){
                // this means that there is enough difference between these points to count
                // as subgoal points...

                if(sphere_matrix[m][n] < 0.001){
                    //this means we are at the edge of obstacle

                    // KEEP WORKING FROM HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                    phi = (m+0.5) * pi / M - pi/2; // angle at m
                    theta = (n+0.5) *  pi / M - pi;
                    rho = sphere_matrix[m+1][n] + robot_radius*2;
                    r = rho * cos(phi);
                    x = r * cos(theta);
                    y = r * sin(theta);
                    z = rho * sin(phi);
                    point_robot_frame[0] = x;
                    point_robot_frame[1] = y;
                    point_robot_frame[2] = z;

                    point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                    2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                    point[0] += robot_position[0];
                    point[1] += robot_position[1];
                    point[2] += robot_position[2];

                    angular_offset = ceil(angular_offset_factor / sphere_matrix[m+1][n]);
                    if(m+1-angular_offset>-1){ 
                        //subgoal_matrix[m+1-angular_offset][n] = sphere_matrix[m+1][n] + robot_radius*2;// + safety_distance;  original...
                        if(tree->search(point[0], point[1],point[2])){
                            if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m+1-angular_offset][n] = sphere_matrix[m+1][n] + robot_radius*2; 
                        }

                    }
                }
                else if(sphere_matrix[m+1][n] < 0.001){
                    //this means we are at the edge of obstacle
                    phi = (m+1.5) * pi / M - pi/2; // angle at m+1
                    theta = (n+0.5) *  pi / M - pi;
                    rho = sphere_matrix[m][n] + robot_radius*2;
                    r = rho * cos(phi);
                    x = r * cos(theta);
                    y = r * sin(theta);
                    z = rho * sin(phi);
                    point_robot_frame[0] = x;
                    point_robot_frame[1] = y;
                    point_robot_frame[2] = z;

                    point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                    2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                    point[0] += robot_position[0];
                    point[1] += robot_position[1];
                    point[2] += robot_position[2];


                    angular_offset = ceil(angular_offset_factor / sphere_matrix[m][n]);
                    if(m+angular_offset<M){ 
                        //subgoal_matrix[m+angular_offset][n] = sphere_matrix[m][n] + robot_radius*2;// + safety_distance;
                        if(tree->search(point[0], point[1],point[2])){
                            if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m+angular_offset][n] = sphere_matrix[m][n] + robot_radius*2;
                        }   
                    }
                } 
                else if(difference > 2*robot_radius){
                    //otherwise we're between obstacles

                    // Check here if point is located on surface ..
                    phi = (m+1) * pi / M - pi/2;// + deg_resolution/2 * pi/180; angle between m and m+1, effectively between m+0.5 and m+1.5
                    theta = (n+0.5) *  pi / M - pi;


                    if(sphere_matrix[m][n]<sphere_matrix[m+1][n]){

                        rho = sphere_matrix[m][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));
                        angular_offset = ceil(angular_offset_factor / sphere_matrix[m][n]);
                        if(m+angular_offset<M){
                            // if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m+angular_offset][n] = rho-resolution/2;
                            // else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m+angular_offset][n] = rho-resolution/2;
                            if(tree->search(point[0], point[1],point[2])){
                                if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m+angular_offset][n] = rho-resolution/2; 
                            }
                        }
                    }
                    else{

                        rho = sphere_matrix[m+1][n] + difference/2 + resolution/2;
                        r = rho * cos(phi);
                        x = r * cos(theta);
                        y = r * sin(theta);
                        z = rho * sin(phi);
                        point_robot_frame[0] = x;
                        point_robot_frame[1] = y;
                        point_robot_frame[2] = z;

                        point = point_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, point_robot_frame) +
                        2*cross(robot_orientation_v, cross(robot_orientation_v, point_robot_frame));

                        point[0] += robot_position[0];
                        point[1] += robot_position[1];
                        point[2] += robot_position[2];

                        //points_checked_msg->points.push_back(pcl::PointXYZ(point[0], point[1],point[2]));
                        angular_offset = ceil(angular_offset_factor / sphere_matrix[m+1][n]);
                        if(m+1-angular_offset>-1){
                            // if(!tree->search(point[0], point[1],point[2])) subgoal_matrix[m+1-angular_offset][n] = rho-resolution/2;
                            // else if(tree->search(point[0], point[1],point[2]) < 0) subgoal_matrix[m+1-angular_offset][n] = rho-resolution/2;
                            if(tree->search(point[0], point[1],point[2])){
                                if(tree->search(point[0], point[1],point[2])->getValue() < 0) subgoal_matrix[m+1-angular_offset][n] = rho-resolution/2;
                            }
                        }
                    }
                }
                //if(angular_offset < 1) cout << "angular offset is: " << angular_offset << " in m difference" << endl;
                //cout << "angular_offset for m: " << angular_offset << endl;
            }
            //}
        }
    }


/*
    points_checked_msg->width = points_checked_msg->size();
    // Convert to ROS data type
    sensor_msgs::PointCloud2 points_checked_output;
    pcl::toROSMsg(*points_checked_msg, points_checked_output);
    // Publish the data
    pub_points_checked_cloud.publish (points_checked_output);
  */  

    /*std_msgs::Float32MultiArray dat1;
    int H = M;
    int W = N;
     // fill out message:
    dat1.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat1.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat1.layout.dim[0].label = "height";
    dat1.layout.dim[1].label = "width";
    dat1.layout.dim[0].size = H;
    dat1.layout.dim[1].size = W;
    dat1.layout.dim[0].stride = H*W;
    dat1.layout.dim[1].stride = W;
    dat1.layout.data_offset = 0;
    std::vector<float> vec(W*H, 0);
    for (int i=0; i<H; i++)
        for (int j=0; j<W; j++)
            vec[i*W + j] = subgoal_matrix[i][j];
    dat1.data = vec;

    pub_subgoal_matrix.publish(dat1);*/

    std_msgs::Float32MultiArray dat;
    int H = M*3;
    int W = N;
     // fill out message:
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "height";
    dat.layout.dim[1].label = "width";
    dat.layout.dim[0].size = H;
    dat.layout.dim[1].size = W;
    dat.layout.dim[0].stride = H*W;
    dat.layout.dim[1].stride = W;
    dat.layout.data_offset = 0;
    std::vector<float> vec(W*H, 0);
    for (int i=0; i<M; i++)
        for (int j=0; j<W; j++)
            //vec[i*W + j] = sphere_matrix[i][j];
            vec[i*W + j] = sphere_matrix_with_unknown[i][j];
    for (int i=M; i<M*2; i++)
        for (int j=0; j<W; j++)
            vec[i*W + j] = subgoal_matrix[i-M][j];
    for (int i=M*2; i<M*3; i++)
        for (int j=0; j<W; j++)
            vec[i*W + j] = sphere_matrix[i-M*2][j];
        
    dat.data = vec;

    pub_matrix.publish(dat);





    //-------------------Transform again to point cloud for visualizing in rviz:------------------------

    //double x, y, z, r, rho, phi, theta;

    /*PointCloud::Ptr subgoal_msg (new PointCloud);
    subgoal_msg->header.frame_id = "base_link";
    subgoal_msg->height = 1;
    subgoal_msg->width = M*N;

      //for(int i = 0, max = 120*60; i!=max, i++) {
    for( m=0; m < M; m++){
        for(n=0; n<N; n++){
            if(subgoal_matrix[m][n] == 0){
                subgoal_msg->points.push_back (pcl::PointXYZ(0,0,0));
            }
            else
            {
                //r = rho * sin(phi);
                rho = subgoal_matrix[m][n];
                theta = n *  pi / M - pi;
                phi = m * pi / M - pi/2;

                r = rho * cos(phi);             
                x = r * cos(theta);
                y = r * sin(theta);
                z = rho * sin(phi);
                subgoal_msg->points.push_back (pcl::PointXYZ(x,y,z));
                //cout << "x: " << x << endl;
            }
        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 subgoal_output;
    pcl::toROSMsg(*subgoal_msg, subgoal_output);
    // Publish the data
    pub_subgoal_cloud.publish (subgoal_output);*/

}


void OctomapToSpherical::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input) {

    robot_position[0] = input->pose.position.x;
    robot_position[1] = input->pose.position.y;
    robot_position[2] = input->pose.position.z;

    //cout << "robot position: " << robot_position << endl;

    robot_orientation[0] = input->pose.orientation.x;
    robot_orientation[1] = input->pose.orientation.y;
    robot_orientation[2] = input->pose.orientation.z;
    robot_orientation[3] = input->pose.orientation.w;

}

//member function implementation for a service callback function
/*bool OctomapToSpherical::serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}*/

Vector3d OctomapToSpherical::cross(const Vector3d & vec1, const Vector3d & vec2){
    Vector3d cross_product;
    cross_product[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    cross_product[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    cross_product[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    
    return ( cross_product );
}

float OctomapToSpherical::dot(const Vector3d & vec1, const Vector3d & vec2){
    
    return ( vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2] );
}

/*tf::Vector3 OctomapToSpherical::tf_multiply(Quaternion quat, Vector3 vec){
     float num = quat.x * 2f;
     float num2 = quat.y * 2f;
     float num3 = quat.z * 2f;
     float num4 = quat.x * num;
     float num5 = quat.y * num2;
     float num6 = quat.z * num3;
     float num7 = quat.x * num2;
     float num8 = quat.x * num3;
     float num9 = quat.y * num3;
     float num10 = quat.w * num;
     float num11 = quat.w * num2;
     float num12 = quat.w * num3;
     Vector3 result;
     result.x = (1f - (num5 + num6)) * vec.x + (num7 - num12) * vec.y + (num8 + num11) * vec.z;
     result.y = (num7 + num12) * vec.x + (1f - (num4 + num6)) * vec.y + (num9 - num10) * vec.z;
     result.z = (num8 - num11) * vec.x + (num9 + num10) * vec.y + (1f - (num4 + num5)) * vec.z;
     return result;
 }*/



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "octomapToSpherical"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type OctomapToSpherical");
    OctomapToSpherical octomapToSpherical(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 