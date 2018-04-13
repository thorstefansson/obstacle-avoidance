#include <iostream>
#include <cmath>
#include "math.h"




// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "sensors_to_spherical.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
SensorsToSpherical::SensorsToSpherical(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of SensorsToSpherical");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    
    //initialize variables here, as needed
    val_to_remember_=0.0;
    pi = 3.14159265359;
    sin22komma5 = sin(22.5*pi/180);
    //sonar_up_offset[3] = {-0.05, 0.0, 0.13};
    //sonar_down_offset[3] = {-0.05, 0.0, -0.05};
    sonar_up_vertical_offset = 0.13;
    sonar_down_vertical_offset = -0.05;
    max_camera_range = 10; //it's actually 1.4 m but 10m cameras exist...
    camera_x_offset = 0.05;
    robot_radius = 0.2;
    safety_distance = 0.3;
    max_sonar_range = 6;
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void SensorsToSpherical::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    // add more subscribers here, as needed
    camera_subscriber_ = nh_.subscribe("/voxelpoints", 1, &SensorsToSpherical::cameraCallback, this);    //<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
    sonar_up_subscriber_ = nh_.subscribe("/sonar_up/range", 1, &SensorsToSpherical::sonarupCallback, this);    //<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
    sonar_down_subscriber_ = nh_.subscribe("/sonar_down/range", 1, &SensorsToSpherical::sonardownCallback, this);
    lidar_cloud_subscriber_ = nh_.subscribe("/laser_cloud", 1, &SensorsToSpherical::lidarcloudCallback, this);

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
void SensorsToSpherical::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 

    pub = nh_.advertise<sensor_msgs::PointCloud2>("spherical_cloud", 1, true);
    pub_matrix = nh_.advertise<std_msgs::Float32MultiArray>("spherical_matrix", 1, true);
    pub_sonar_degree_limits_and_ranges = nh_.advertise<std_msgs::Int16MultiArray>("sonar_degree_limits", 1, true);

    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



/*
// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
void ExampleRosClass::subscriberCallback(const std_msgs::Float32& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the square of the received value; 
}*/

void SensorsToSpherical::sonarupCallback(const sensor_msgs::RangeConstPtr& input) {

	range_up = input->range;
	//cout << "range up:" <<range_up << endl;
}

void SensorsToSpherical::sonardownCallback(const sensor_msgs::RangeConstPtr& input) {
	range_down = input->range;
}


void SensorsToSpherical::lidarcloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::fromROSMsg (*input, lidarcloud);
	// cout << "in lidar callback" << endl;

}

void SensorsToSpherical::cameraCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);

	double x;
	double y;
	double z;
	double r, rho, phi, theta;
	//double pi = 3.14159265359;

  // If we use boxes with 3 degrees width and height, there are m=60 * n=120 sections

	int M = 60;
	int N = 120;

	double sphere_matrix [M][N] = {0};

	//For some reason, for the camera in the camera_link frame, z is forward, y down and x to the right. 
	// weird.
	//cout << "in function" << endl;
	int m,n;

	/*int max_m, max_n, min_m, min_n;
	max_m = 35;
	min_m = 35;
	max_n = 60;
	min_n = 60;*/
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud.begin(); it != cloud.end(); it++){

		if(!isnan(it->y)){
			//Since z is forward, y down and x to the right:
			// and we want x forward, y left and z up:
			y = -(it->x);
			z = -it->y + 0.13;
			x = it->z + 0.05;


			//If x is forward, y left and z up:
			/*
			//transform from camera to base link:
			x = it->x - 0.05;
			y = it->y;
			z = it->z - 0.13;
			*/

			//transform to spherical coordinates:
			r = sqrt( pow(x, 2) + pow(y,2));
			rho = sqrt( pow(r,2) + pow(z, 2)); 	//distance from base link
			theta = asin(y / r);			//angle from x (forward) axis in xy plane
			phi = asin(z / rho);			//angle from xy plane	 

			// all right so now hopefully theta is between -pi - pi and phi between
			// - pi/2 - pi/2
			
			//find potential placement in matrix
			m = floor((phi + pi/2) * M / pi);
			n = floor((theta + pi) * M / pi);
			//cout << "m:" << m << "n:" << n << endl;

			if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
				if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] ==0){
					//cout << "in" << endl;
					sphere_matrix[m][n] = rho;	
			}
			}
			else{
				cout << "m: " << m << " n:" << n << endl;
			}

			//let's find the limits of the camera:
			//looks like it's like this: min m: 27 max m: 40 min n: 50 max_n: 69
			/*
			if(m>max_m) max_m = m;
			if(n>max_n) max_n = n;
			if(n<min_n) min_n = n;
			if(m<min_m) min_m = m;*/
		}
	}

	//cout << "min m: " << min_m << " max m: " << max_m << " min n: " << min_n << " max_n: " << max_n << endl;
	//so fill up the cloud in the camera view where there is no obstacle seen with the range of camera

	for( m = 28 ; m < 39 ; m++){  //max is like 27-41
		for( n = 51; n< 69 ; n++){ //and 50 - 68
			if(sphere_matrix[m][n] == 0) sphere_matrix[m][n] = max_camera_range + camera_x_offset;
		}
	}


	//cout << "well now" << endl;
	//let's do a similar thing for the lidar cloud:
	//int infcount = 0;

	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = lidarcloud.begin(); it != lidarcloud.end(); it++){

		//out <<"hey" << endl;
		//if(!isnan(it->y)){ //seems to be no lidar nans ...


		//If x is forward, y left and z up:
		
		//transform from camera to base link:
		x = it->x + 0.02;
		y = it->y;
		z = it->z + 0.08;
		
		
		//transform to spherical coordinates:
		r = sqrt( pow(x, 2) + pow(y,2));
		rho = sqrt( pow(r,2) + pow(z, 2)); 	//distance from base link
		theta = asin(y / r);			//angle from x (forward) axis in xy plane
		phi = asin(z / rho);			//angle from xy plane	 

		// all right so now hopefully theta is between -pi - pi and phi between
		// - pi/2 - pi/2
		
		//find potential placement in matrix
		m = floor((phi + pi/2) * M / pi);
		n = floor((theta + pi) * M / pi);
		//cout << "m:" << m << "n:" << n << endl;

		if(m> -1 && n> -1 && m < M && n< N){ //~isnan(m) && ~isnan(n)){
			if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] ==0){
				//cout << "in" << endl;
				sphere_matrix[m][n] = rho;	
		}
		}
		else{
			cout << "out of bounds: ";
			cout << "m: " << m << " n:" << n << endl;
		}

		/*}
		else{
			nancount++;
		}*/
	}

	//cout << "size of lidarcloud: " << lidarcloud.width * lidarcloud.height << endl;

	//cout << "count of lidar nan: " << nancount << endl; seems to be no lidar nans ...




	//Now add sonar data to sphere matrix

	//publish degree limit to topic:
	std_msgs::Int16MultiArray degree_limits;

	degree_limits.data.clear();

	//First to account for vertical offset:

	//ah let's only account for vertical offset for now...
	//sonar up:
	double cone_up_degrees = asin(range_up * sin22komma5/ (sonar_up_vertical_offset + range_up));
	int degree_limit = M - ceil(cone_up_degrees * M/pi) - 1;
	degree_limits.data.push_back(degree_limit);
	degree_limits.data.push_back(sonar_up_vertical_offset + range_up);

	for (m = degree_limit; m<M ; m++){
		for(n=0;n<N;n++){
			sphere_matrix[m][n]=sonar_up_vertical_offset + range_up;
		}
	}

	double cone_down_degrees = asin(range_down * sin22komma5/ (sonar_down_vertical_offset + range_down));
	degree_limit = ceil(cone_down_degrees * M/pi)-1;
	degree_limits.data.push_back(degree_limit);
	degree_limits.data.push_back(-sonar_down_vertical_offset + range_down);
	
	for (m = 0; m<degree_limit ; m++){
		for(n=0;n<N;n++){
			sphere_matrix[m][n]= -sonar_down_vertical_offset + range_down;
		}
	}

    pub_sonar_degree_limits_and_ranges.publish(degree_limits);



	//add radius + safety distance of robot to cloud where there is no info:

	for (m = 0; m<M ; m++){
		for(n=0;n<N;n++){
			if (sphere_matrix[m][n]==0) sphere_matrix[m][n] = robot_radius + safety_distance;
		}
	}


/*
	cout << "original matrix: " << endl;// << sphere_matrix << endl;
	for (m = 0; m<M ; m++){
		for(n=0;n<N;n++){
			cout << sphere_matrix[m][n] << " " ;
		}
		cout << endl;
	}	
*/
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


 /*
	//-------------------Transform again to point cloud for visualizing in rviz:------------------------

	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "base_link";
	msg->height = 1;
	msg->width = 60*120;

	  //for(int i = 0, max = 120*60; i!=max, i++) {
	for( m=0; m < 60; m++){
		for(n=0; n<120; n++){
			if(sphere_matrix[m][n] ==0){
				msg->points.push_back (pcl::PointXYZ(0,0,0));
			}
			else
			{
				//r = rho * sin(phi);
				rho = sphere_matrix[m][n];
				theta = n *  pi / 60 - pi;
				phi = m * pi / 60 - pi/2;

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
  	pub.publish (output); */

}

/*
//member function implementation for a service callback function
bool ExampleRosClass::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}*/



int main(int argc, char** argv)
{

// ROS set-ups:
    ros::init(argc, argv, "sensors_to_spherical"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type SensorsToSpherical");
    SensorsToSpherical sensorsToSpherical(&nh);  //instantiate an SensorsToSpherical object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
	return 0;



	/*
  ros::init(argc, argv, "sensors_to_spherical");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1.0);
  Sensors_to_spherical rd(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;*/
}