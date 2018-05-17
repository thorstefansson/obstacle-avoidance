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
    resolution = 0.08;
    pi = 3.14159265359;
    initializeOctomap();
    box_width_m = 8.0;
    
    // Height and width of our spherical occupancy matrix:
    // for 3 degrees resolution: M = 60;N = 120;

    deg_resolution = 6;
    M = 180 / deg_resolution;
    N = 360 / deg_resolution;

    half_box_width_m = box_width_m/2;
    half_box_width_m_sq = pow(half_box_width_m, 2);
    // can also do tests/waits to make sure all required services, topics, etc are alive
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
    pub_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/bounding_box", 1, true);
    pub_init_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_to_octomap", 1, true);

    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

void OctomapToSpherical::initializeOctomap()
{
    // initialize the octomap in here..

    // First create point cloud that we publish to the topic that changes octomap,
    //to make a free box or sphere around the initial robot position

    double width = 20*resolution;
    double robot_height_from_ground = 0.05;

    PointCloud::Ptr cloud_msg (new PointCloud);
    cloud_msg->header.frame_id = "base_link";
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
        for (float z = robot_height_from_ground; z < width + robot_height_from_ground; z += resolution)
        {
            cloud_msg->points.push_back (pcl::PointXYZ(-width/2,y,z));  // ? how high is base_link above ground??
            cloud_msg->points.push_back (pcl::PointXYZ(width/2,y,z));
            count+=2;
        }
    }
    // Fill in side where y constant
    for(float x = -width/2 ; x < width/2 ; x += resolution){
        for (float z = robot_height_from_ground; z < width + robot_height_from_ground; z += resolution)
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
    while (count < 20)
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


    point3d min;
    min.x() = robot_position[0] - half_box_width_m; min.y() = robot_position[1] - half_box_width_m; min.z() = robot_position[2] - half_box_width_m;
    //cout << "min"<< min << endl;
    point3d max; 
    max.x() = robot_position[0] + half_box_width_m; max.y() = robot_position[1] + half_box_width_m; max.z() = robot_position[2] + half_box_width_m;
    //cout << "max" <<max << endl;

    int box_width = box_width_m/resolution;
    //cout << "box_width = " << box_width << endl;
    //int bounding_box_matrix [box_width] [box_width] [box_width] = {0};
    //cout << "Hmm" << endl;

    //cout << "box_width: " << box_width << endl;

    // OR:
/*
    int countx=0, county=0, countz=0;
    for (double ix = min.x() + resolution/2; ix < max.x(); ix += resolution){
        for (double iy = min.y() + resolution /2; iy < max.y(); iy += resolution){
            for (double iz = min.z()+ resolution /2; iz < max.z(); iz += resolution)
            {
                //cout << "matrix coordinates: " << countx << " " << county << " " countz << " ";
                if (!tree->search(ix,iy,iz))
                {
                    //This cell is unknown
                }
                else if(tree->search(ix,iy,iz)->getValue() < 0){
                    // this cell is free ...
                    bounding_box_matrix [countx] [county] [countz] = -1;
                }
                else if(tree->search(ix,iy,iz)->getValue() > 0){
                    // this cell is occupied
                    bounding_box_matrix [countx] [county] [countz] = 1;
                }
                else{
                    cout << "eitthvad skrytid" << endl;
                }  

                countz++;
            }
            county++;
            countz =0;
        }
        countx++;
        county = 0;
    }

    //-------------------Transform again to point cloud for visualizing in rviz:------------------------

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




    int m,n;
    double r, rho, phi, theta;

    //cout << "here" << endl;
    double sphere_matrix [M][N] = {0};
    double r_sq, distance_sq, node_size;
    int node_size_multiple, size_distance_factor;
    double x,y,z;

    // Now we need to rotate the coordinates so they are in robot frame.....
    // would be good to find some other solution to save computational power.. 

    // To get the point in the same frame as the robot we need to turn the it
    // to do this we find the inverse of the robot quaternion and use it to turn the point

    // inverse of robot quaternion:
    Vector3d robot_orientation_inverse_v;
    float robot_orientation_inverse_w;

    float robot_orientation_sum_squared = pow(robot_orientation[0],2) + pow(robot_orientation[1],2) + pow(robot_orientation[2],2)
    +pow(robot_orientation[3],2);

    robot_orientation_inverse_w = robot_orientation[3] / robot_orientation_sum_squared;

    robot_orientation_inverse_v[0] = -robot_orientation[0] / robot_orientation_sum_squared;
    robot_orientation_inverse_v[1] = -robot_orientation[1] / robot_orientation_sum_squared;
    robot_orientation_inverse_v[2] = -robot_orientation[2] / robot_orientation_sum_squared;
    Vector3d point, point_robot_frame;
    // Use:
    //cout << "2" << endl ; // "going into bbx iterator." << endl;
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

            point_robot_frame = point + 2*robot_orientation_inverse_w*cross(robot_orientation_inverse_v, point) +
            2*cross(robot_orientation_inverse_v, cross(robot_orientation_inverse_v, point));
            
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
                n = ceil((theta + pi) * M / pi);       // should look further into this....................
                //cout << "m:" << m << "n:" << n << endl;

                if(n>=N){
                    n=n-N;
                }
                if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
                    if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] ==0){
                        //cout << "in" << endl;
                        sphere_matrix[m][n] = rho;  
                    }
                }
                else{
                    cout << "m: " << m << " n:" << n << endl;
                }

                //since we have boxes of size 3 deg, at 1m distance that is 5 cm, which is the resolution of the octomap...
                // so .. 
                node_size_multiple = round(node_size/resolution);
                if(rho<1 || node_size_multiple >1 ){
                    //cout << "special case" << endl;
                    // now we need to fill out more than one element in the sphere matrix
                    size_distance_factor =  floor(4 * node_size_multiple / ceil(rho*4)) ;
                    
                    if(size_distance_factor >=1){ // size_distance_factor >=1
                        
                        /*// find which quadrant of angle box the center is..
                        bool horizontal = signbit((phi + pi/2) * M / pi round((phi + pi/2) * M / pi));*/
                        // ah, fuck that, lets just make this simple... and perhaps fill out more than we should.. 

                        for(int i = m - size_distance_factor ; i <= m+size_distance_factor ; i++){
                            for(int j = n - size_distance_factor ; j <= n+size_distance_factor ; j++){
                                if(i> -1 && j> -1 && i < M && j< N){ //?
                                    if( rho < sphere_matrix[i][j] || sphere_matrix[i][j] ==0){
                                            sphere_matrix[i][j] = rho;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

       /*if(it->getValue() < 0 ){
        // means node is free I think

        cout << "coordinates in matrix: " << (int) ((it.getX() - robot_position[0] + 3.0) / resolution - 1) <<" "
        <<(int) ((it.getY() - robot_position[1] + 3.0) / resolution -1) << " " << (int) ((it.getZ() - robot_position[2] + 3.0) / resolution -1) << endl;

        bounding_box_matrix [(int) ((it.getX() - robot_position[0] + 3.0) / resolution - 1)] [(int) ((it.getY() - robot_position[1] + 3.0) / resolution -1)]
        [(int) ((it.getZ() - robot_position[2] + 3.0) / resolution -1)] = -1;
       }
       else{
        // means node is occupied
       }*/
       //if (it.getSize() > 0.05 ) cout << "larger" ;
    }

    // We want to publish the position and orientation of the robot at the time of this matrix being calculated to make accurate sub goal distance computations

    // so let's allocate some space in the sphere matrix for that.. 
    sphere_matrix[0][0] = robot_position[0];
    sphere_matrix[0][1] = robot_position[1];
    sphere_matrix[0][2] = robot_position[2];
    sphere_matrix[0][3] = robot_orientation[0];
    sphere_matrix[0][4] = robot_orientation[1];
    sphere_matrix[0][5] = robot_orientation[2];
    sphere_matrix[0][6] = robot_orientation[3];


        //---------------------------------Publish matrix to topic:-----------------------------------------

    std_msgs::Float32MultiArray dat;
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

    pub_matrix.publish(dat);


 
    //-------------------Transform again to point cloud for visualizing in rviz:------------------------


    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "base_link";
    msg->height = 1;
    msg->width = M*N;

      //for(int i = 0, max = 120*60; i!=max, i++) {
    for( m=0; m < M; m++){
        for(n=0; n<N; n++){
            if(sphere_matrix[m][n] == 0){
                msg->points.push_back (pcl::PointXYZ(0,0,0));
            }
            else
            {
                //r = rho * sin(phi);
                rho = sphere_matrix[m][n];
                theta = n *  pi / M - pi;
                phi = m * pi / M - pi/2;

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