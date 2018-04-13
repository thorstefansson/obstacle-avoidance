// template from:      https://github.com/wsnewman/learning_ros/tree/master/Part_1/example_ros_class/src

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "goal_direction.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
GoalDirection::GoalDirection(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of GoalDirection");
    // Sleep for 2 seconds to wait for everything else to start up:
    //sleep(200);
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    
    //initialize variables here, as needed
    //val_to_remember_=0.0;
    pi = 3.14159265359;

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
    spherical_matrix_height = 60;
    spherical_matrix_width = 120;

    Cspaceframe_activepoints = 0;

	cspace_resolution = 0.02; // meters
	cspace_width = ceil(robot_diameter / cspace_resolution);
	cspace_half_width = ceil(robot_radius / cspace_resolution);

	// just initialize as something the following:

	n_radius = 10;//robot_radius / cspace_resolution;
	// for robot radius 0.2 and cspace_rosolution 0.02 n_radius is 10
	//sphere_model[n_radius][n_radius][n_radius] = {0};
	//Cspaceframe [cspace_width] [cspace_width] = {0};

	// just initialize as something the following:
	sonar_up_limit = 6;
	range_up = 6;
	sonar_down_limit = 6;
	range_down = 0;

	cout <<"Going into initialize Sphere function" << endl;
	initializeSphere();
	cout <<"done running initializeSphere" << endl;


    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void GoalDirection::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    // add more subscribers here, as needed
    //camera_subscriber_ = nh_.subscribe("/camera/depth/points", 1, &GoalDirection::cameraCallback, this); 
    goal_position_sub_ = nh_.subscribe("/goal_position", 1, &GoalDirection::goalPositionCallback, this);
	occupancy_matrix_sub_ = nh_.subscribe("/spherical_matrix", 1, &GoalDirection::sphericalMatrixCallback, this);
	sonar_limits_and_ranges_sub_ = nh_.subscribe("/sonar_degree_limits", 1, &GoalDirection::sonarlimitsrangesCallback, this);
	camera_subscriber_ = nh_.subscribe("/voxelpoints", 1, &GoalDirection::cameraCallback, this);
	robot_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &GoalDirection::robotPositionCallback, this);
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

void GoalDirection::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 
    pub = nh_.advertise<sensor_msgs::PointCloud2>("spherical_cloud", 1, true);
    pub_Cspace = nh_.advertise<sensor_msgs::PointCloud2>("Cspace_cloud", 1, true);
    pub_subgoal_cloud= nh_.advertise<sensor_msgs::PointCloud2>("subgoal_cloud", 1, true);
    pub_selected_subgoal = nh_.advertise<sensor_msgs::PointCloud2>("selected_subgoal", 1, true);

    pub_desired_position_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1, true);

}

void GoalDirection::initializeSphere()
{
	// Calculate the position of occupied points resulting from a single obstacle point:
	// it is a cartesian discretized sphere:

	double jvalue, kvalue;

	//ivalue = cspace_resolution;
	//jvalue = cspace_resolution;


	//cout << "in initializeSphere function" << endl;
	/*cout << "n_radius: " << n_radius << endl;
	cout << "cspace_width / 2: " << cspace_width/2 << endl;
	cout << "robot radius: " << robot_radius << endl; 
	/*cout << "cspace_width:" << cspace_width << endl;
	cout << "cspace_half_width" << cspace_half_width << endl;*/

	double ivalue = 0;
	for (int i = 0 ; i < n_radius ; i++){
		//jvalue=0;
		//ivalue = i*cspace_resolution;
		for ( int j = i ; j < n_radius ; j++){
			//kvalue=0;
			jvalue = cspace_resolution*j;
			for (int k = j ; k < n_radius; k ++){
				kvalue = cspace_resolution*k;

				/*cout << "Here1 ivalue: " << ivalue << "jvalue: " << jvalue <<"kvalue: "<<kvalue<<endl;
				bool statement = ivalue+jvalue+kvalue < robot_radius;
				cout << "statement: " << ivalue+jvalue+kvalue << " < " << robot_radius << endl;*/
				//scout << "outcome of statement: " << statement << endl;
				//cout << "bool of statement:" << bool(statement) << endl;
				if(ivalue+jvalue+kvalue < robot_radius){
					// This is within the sphere
					sphere_model[i][j][k] = 1;
					sphere_model[i][k][j] = 1;
					sphere_model[j][i][k] = 1;
					sphere_model[j][k][i] = 1;
					sphere_model[k][i][j] = 1;
					sphere_model[k][j][i] = 1;
					if(i==0){
							// fill in frame

							Cspaceframe[k + cspace_width/2][j + cspace_width/2] = 0;
							Cspaceframe[j + cspace_width/2][k + cspace_width/2] = 0;

							Cspaceframe[k + cspace_width/2][cspace_width/2-j-1] =0;
							Cspaceframe[cspace_width/2-j-1][k + cspace_width/2] =0;

							Cspaceframe[j + cspace_width/2][cspace_width/2-k-1]=0;
							Cspaceframe[cspace_width/2-k-1][j + cspace_width/2]=0;

							Cspaceframe[ cspace_width/2-k-1][ cspace_width/2-j-1] = 0;
							Cspaceframe[ cspace_width/2-j-1][ cspace_width/2-k-1] = 0;
							if(j !=k) Cspaceframe_activepoints += 8;
							else Cspaceframe_activepoints += 4;

					}
				}
				else{

					/*cout << "Here2 ivalue: " << ivalue << "jvalue: " << jvalue <<"kvalue: "<<kvalue<<endl;
					cout << "statement: " << pow (ivalue, 2) + pow (jvalue, 2) + pow(kvalue,2) <<" < " << radius_sq << endl;*/
					if( pow (ivalue, 2) + pow (jvalue, 2) + pow(kvalue,2) < radius_sq){
						//also within sphere
						sphere_model[i][j][k] = 1;
						sphere_model[i][k][j] = 1;
						sphere_model[j][i][k] = 1;
						sphere_model[j][k][i] = 1;
						sphere_model[k][i][j] = 1;
						sphere_model[k][j][i] = 1;

						if(i==0){
							// fill in frame
							Cspaceframe[k + cspace_width/2][j + cspace_width/2] = 0;
							Cspaceframe[j + cspace_width/2][k + cspace_width/2] = 0;
							Cspaceframe[k + cspace_width/2][cspace_width/2-j-1] =0;
							Cspaceframe[cspace_width/2-j-1][k + cspace_width/2] =0;
							Cspaceframe[j + cspace_width/2][cspace_width/2-k-1]=0;
							Cspaceframe[cspace_width/2-k-1][j + cspace_width/2]=0;
							Cspaceframe[ cspace_width/2-k-1][ cspace_width/2-j-1] = 0;
							Cspaceframe[ cspace_width/2-j-1][ cspace_width/2-k-1] = 0;
							if(j !=k) Cspaceframe_activepoints += 8;
							else Cspaceframe_activepoints += 4;
						}
					}
					else{
						// outside sphere
						sphere_model[i][j][k] = 0;
						sphere_model[i][k][j] = 0;
						sphere_model[j][i][k] = 0;
						sphere_model[j][k][i] = 0;
						sphere_model[k][i][j] = 0;
						sphere_model[k][j][i] = 0;

						//cout << "Here3 i: " << i << "j: " << j <<"k: "<<k<<endl;

						if(i == 0){
							//Cspaceframe[i][j] = 1;
							// fill in frame
							cout << "j: "<< j <<" k: " <<k <<  endl;
							Cspaceframe[k + cspace_width/2][j + cspace_width/2] = 1;
							Cspaceframe[j + cspace_width/2][k + cspace_width/2] = 1;
							Cspaceframe[k + cspace_width/2][cspace_width/2-j-1] =1;
							Cspaceframe[cspace_width/2-j-1][k + cspace_width/2] =1;
							Cspaceframe[j + cspace_width/2][cspace_width/2-k-1]=1;
							Cspaceframe[cspace_width/2-k-1][j + cspace_width/2]=1;
							Cspaceframe[ cspace_width/2-k-1][ cspace_width/2-j-1] = 1;
							Cspaceframe[ cspace_width/2-j-1][ cspace_width/2-k-1] = 1;
							//Cspaceframe_activepoints += 8;
						}
					}
				}
			}
			//start at 2cm (cspace resolution)
			//end at robot radius...
		}
		ivalue += cspace_resolution;
	}

	//lets write out our frame:

	cout <<"Cspaceframe: " << endl;

	for(int i = 0; i < cspace_width ; i++){
		for (int j = 0; j<cspace_width; j++){
			cout<< Cspaceframe[i][j];
		}
		cout << endl;
	}

	cout << "Cspaceframe_activepoints: "  << Cspaceframe_activepoints  << endl;
}

void GoalDirection::sonarlimitsrangesCallback(const std_msgs::Int16MultiArray::ConstPtr& input){
	sonar_up_limit = input -> data[0];
	range_up = input -> data[1];
	sonar_down_limit = input -> data[2];
	range_down = input -> data[3];

	//cout << "sonar up limit: " << sonar_up_limit << "sonar down limit: " 
	//<< sonar_down_limit << "range_up: " << range_up << "range_down: " << range_down << endl;
}


void GoalDirection::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){
//double [60][120] GoalDirection::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){

	//cout << "in sphericalMatrixCallback" << endl;

	float dstride0 = matrix_msg->layout.dim[0].stride;
	float dstride1 = matrix_msg->layout.dim[1].stride;
	float h = matrix_msg->layout.dim[0].size;
	float w = matrix_msg->layout.dim[1].size;
	int M = h;
	int N = w;

	/*
		float w = msg->layout.dim[1].size;
	// Below are a few basic Eigen demos:
	//std::vector<float> data = matrix_msg->data;
	//Eigen::Map<Eigen::MatrixXf> mat(data.data(), h, w);
*/
	//anyways,  let's try converting this to array:

	int n,m;
	//double sphere_matrix [M][N] = {0}; defined in .h file
	for (m = 0; m< M ; m++){
		for (n = 0; n< N ; n++){
			sphere_matrix[m][n] = matrix_msg->data[m*dstride1 + n];
			subgoal_matrix[m][n] = 0;
			} 
	}

	/*
	cout << "result matrix: " << endl;// << sphere_matrix << endl;
	for (m = 0; m<M ; m++){
		for(n=0;n<N;n++){
			cout << sphere_matrix[m][n] << " " ;
		}
		cout << endl;
	}	
*/

	//-------------------Transform again to point cloud for visualizing in rviz:------------------------

	double x, y, z, r, rho, phi, theta;

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
  	pub.publish (output);



  //__________________________________________________________________________________________________

  	//------------------------------------Locate sub goals:-------------------------------------------

  	// okay, so now: where we have no sensor info we put 0.5 m (radius + safety distance) 
  	// the maximum distance the sonars see is 6 m so if they show a greater distance it is set to 
  	// 6 m + vertical offset of sonars, giving 6.13 m up and 6.05m down (sould be more down but whatever)
  	// then for the camera we have maximum range 10m + the offset in x direction which is 5 cm so 10.05m
  	// so in at those distances in those directions we don't actually see obstacles. 
  	// At the same time this implementation should not have to be greatly altered to use with other sensors..

  	//anyways, we should use case 1. below if we see two obstacles at different distances with camera,
  	// and maybe if we see obstacles with sonars..?
  	// otherwise case two..
  	//what do I do with the lidar data then...
  	//Should use all sensor data available to choose a direction probably.. 
  	//maybe find sub goals with lidar in 2D separately
  	//and then integrate.. 
  	//so start by using camera and sonars 

  	// they can be:
  	// 1. In  the middle  point between  two  obstacle  points,  whose  distance  is  greater 
  	//than  the  robot’s  diameter,  when  the  corresponding  sectors  are contiguous

  	// or:
  	//2. In  the  direction  of  the  edge  of  the  obstacle,  at  a  distance  greater  than  the
	//robot’s diameter, when the corresponding sector does not have any contiguous sectors

  	//this matrix contains locations of possible subgoals:
  	//double subgoal_matrix[M][N] = {0};


  	cout  << "Locating subgoals line 360. " << endl;
  	//here I'm assuming that I want to move the robot up/down by the distance of its diameter
  	// if that will be chosen as the most desireable direction...
  	//maybe this should be done in some more clever way
  	if (range_up > robot_radius*3 + safety_distance){
  		for (n = 0 ; n < N ; n++){
  		
  			subgoal_matrix[sonar_up_limit][n] = robot_radius*2;
  		}
  	}
  	if (range_down > robot_radius*3 + safety_distance){
  		for (n = 0 ; n < N ; n++){
  		
  			subgoal_matrix[sonar_down_limit][n] = robot_radius*2;
  		}
  	}
//*/
  	//Now let's add stuff for the camera view... :

  	// we only have to account for area of matrix that is in view of camera
  	//change this if FOV of camera changes !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  	//Brace yourselves, we will have many if statements

  	double difference;

  	//double statement;

  	//cout << "camera_x_offset + max_camera_range : " << camera_x_offset + max_camera_range <<  endl;
	for( m = 25 ; m < 43 ; m++){  //max is like 27-41
		for( n = 48; n< 70 ; n++){ //and 50 - 68

			//To make sure we're not on the edge of view of the camera:
			if(sphere_matrix[m][n] != robot_radius + safety_distance){

				//Check horizontal difference:
				difference = abs(sphere_matrix[m][n]-sphere_matrix[m][n+1]);
				if(difference > 2*robot_radius &&  abs(sphere_matrix[m][n+1] - (robot_radius + safety_distance)) >0.001){ 
				//sphere_matrix[m][n+1] != robot_radius + safety_distance){
					// this means that there is enough difference between these points to count
					// as subgoal points...

					/*cout << "camera_x_offset + max_camera_range : " << camera_x_offset + max_camera_range <<  endl;
					cout << "sphere_matrix[m][n]: " << sphere_matrix[m][n] <<  endl;
					statement = sphere_matrix[m][n] - (camera_x_offset + max_camera_range);
					cout << "statement: " << statement <<  endl;*/
					if(abs(sphere_matrix[m][n] - (camera_x_offset + max_camera_range)) < 0.001){  //just to get rid of some dumb ass floating point error
						//this means we are at the edge of obstacle
						// and sphere_matrix[m][n] in back
						//so case 2 of subgoal location if enough space
						if(difference > robot_radius*3 + safety_distance*2){
							subgoal_matrix[m][n] = sphere_matrix[m][n+1] + robot_radius*2 + safety_distance;
						}
						//otherwise case 1:
						else{
							subgoal_matrix[m][n] = sphere_matrix[m][n+1] + difference / 2;	
						}		 
					}
					else if(abs(sphere_matrix[m][n+1] - (camera_x_offset + max_camera_range))<0.001){
						//this means we are at the edge of obstacle
						//so case 2 of subgoal location if enough space
						if(difference > robot_radius*3 + safety_distance*2){
							subgoal_matrix[m][n+1] = sphere_matrix[m][n] + robot_radius*2 + safety_distance;	
						}
						//otherwise case 1:
						else{
							subgoal_matrix[m][n+1] = sphere_matrix[m][n] + difference / 2;	
						}	 
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

				}

				//Check vertical difference:
				difference = abs(sphere_matrix[m][n]-sphere_matrix[m+1][n]);
				if(difference > 2*robot_radius && abs(sphere_matrix[m+1][n] - (robot_radius + safety_distance))>0.001){
					// this means that there is enough difference between these points to count
					// as subgoal points...

					if(abs(sphere_matrix[m][n] - (camera_x_offset + max_camera_range))<0.001){
						//this means we are at the edge of obstacle
						//so case 2 of subgoal location if enough space
						if(difference > robot_radius*3 + safety_distance*2){
							subgoal_matrix[m][n] = sphere_matrix[m+1][n] + robot_radius*2 + safety_distance;	
						}
						//otherwise case 1:
						else{
							subgoal_matrix[m][n] = sphere_matrix[m+1][n] + difference / 2;	
						}		 
					}
					else if(abs (sphere_matrix[m+1][n] - (camera_x_offset + max_camera_range))< 0.001){
						//this means we are at the edge of obstacle
						//so case 2 of subgoal location if enough space
						if(difference > robot_radius*3 + safety_distance*2){
							subgoal_matrix[m+1][n] = sphere_matrix[m][n] + robot_radius*2 + safety_distance;	
						}
						//otherwise case 1:
						else{
							subgoal_matrix[m+1][n] = sphere_matrix[m][n] + difference / 2;	
						}	 
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
				}
			}
		}
	}



	//-------------------Transform again to point cloud for visualizing in rviz:------------------------

	//double x, y, z, r, rho, phi, theta;

	cout << "transforming subgoals to point cloud line 490. "; //  << endl;
	PointCloud::Ptr subgoal_msg (new PointCloud);
	subgoal_msg->header.frame_id = "base_link";
	subgoal_msg->height = 1;
	subgoal_msg->width = 60*120;

	  //for(int i = 0, max = 120*60; i!=max, i++) {
	for( m=0; m < 60; m++){
		for(n=0; n<120; n++){
			if(subgoal_matrix[m][n] ==0){
				subgoal_msg->points.push_back (pcl::PointXYZ(0,0,0));
			}
			else
			{
				//r = rho * sin(phi);
				rho = subgoal_matrix[m][n];
				theta = n *  pi / 60 - pi;
				phi = m * pi / 60 - pi/2;

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
  	pub_subgoal_cloud.publish (subgoal_output);

  	cout << "done with runnig sphericalMatrixCallback, line 524." ;

}


void GoalDirection::goalPositionCallback(const geometry_msgs::Vector3ConstPtr& input) {

	cout << "in goalPositionCallback, line 531. " ; 

	goal_position[0] = input->x;
	goal_position[1] = input->y;
	goal_position[2] = input->z;
/*
	goal_position.push_back(input->x);
	goal_position.push_back(input->y);
	goal_position.push_back(input->z);
	*/
	//cout<<"here 1"<< endl;
	geometry_msgs::PoseStamped set_pose;

	direction_vector = goal_position - robot_position;

	//cout<<"here 2"<< endl;
	//cout << "robot_vector: " << robot_position[0] << " " << robot_position[1] <<" " << robot_position[2] << endl;
	//cout << "goal_position: " << goal_position[0] << " " << goal_position[1] <<" " << goal_position[2] << endl;

	//cout << "direction_vector: " << direction_vector[0] << " " << direction_vector[1] <<" " << direction_vector[2] << endl;

	double xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));

	
	set_pose.header.frame_id = "map";
	set_pose.header.stamp = ros::Time::now();

	//cout<<"here 3"<< endl;


   vector<distance_and_index> distance_index_vector;
   vector<distance_and_index> :: iterator vitr;
   //vector<pair<int,int>> matrix_indices_vector;
   vector< vector<double> > subgoal_xyz; //indeces_xyz;
   double r, rho, phi, theta;
	
   double x1,y1,z1;

	//cout<<"what's happening"<< endl;
	//if the goal is not directly above or below the robot:
	if(abs(direction_vector[2])/ xy_length_of_direction_vector < 2.8){

		//turn robot in xy plane in direction of goal 

		//find desired degrees in xy plane:
		
		//usually x forward, y left and z up:
		// so maybe do like this:?

		double theta = acos(direction_vector[0]/xy_length_of_direction_vector);
		if(direction_vector[1] < 0){
			theta = -theta;
		}

		//and we want to turn around z axis so our quaternion coordinates become:
		double x= 0;
		double y = 0;
		double z = 1*sin(theta/2);
		double w = cos(theta/2);

		bool reachable;
		//cout << "theta " << theta << endl;
		//cout << "difference in orientation: " << abs(z - robot_orientation[2]) + abs(w - robot_orientation[3]) << endl; 
		//cout << "desired quaternion coordinates: " << "x: " << x << "y: " <<y << "z: " << z <<"w: " <<w << endl;
		//cout << "actual quaternion coordinates: " << "x: " << robot_orientation[0] << "y: " <<robot_orientation[1] << "z: " 
		//<< robot_orientation[2] <<"w: " <<robot_orientation[3] << endl;

		// only check if reachable if within camera limits.. 
		// vertical field of view of camera only 45 degrees so.. 
		cout << "checking if goal within camera limits, line 600. ";
		if((abs(z - robot_orientation[2]) + abs(w - robot_orientation[3])  < 0.05 || abs(z + robot_orientation[2]) + abs(w + robot_orientation[3]) < 0.05)
			&& abs(direction_vector[2])/ xy_length_of_direction_vector < 0.36){ // for within 20 degrees...
			cout << "it is within limits. ";
			//cout << "going into isReachable function" << endl ;
			// To get the direction in the same frame as the robot we need to turn the direction vector
			// to do this we find the inverse of the robot quaternion and use it to turn the direction vector
			// why am I using the inverse? fml

			// inverse of robot quaternion:
			Vector3d robot_orientation_inverse_v;
			float robot_orientation_inverse_w;

			float robot_orientation_sum_squared = pow(robot_orientation[0],2) + pow(robot_orientation[1],2) + pow(robot_orientation[2],2) 
			+pow(robot_orientation[3],2);

			robot_orientation_inverse_w = robot_orientation[3] / robot_orientation_sum_squared;

			robot_orientation_inverse_v[0] = -robot_orientation[0] / robot_orientation_sum_squared;
			robot_orientation_inverse_v[1] = -robot_orientation[1] / robot_orientation_sum_squared;
			robot_orientation_inverse_v[2] = -robot_orientation[2] / robot_orientation_sum_squared;

			Vector3d direction_robot_frame = direction_vector + 2*robot_orientation_inverse_w*cross(robot_orientation_inverse_v, direction_vector) + 
			2*cross(robot_orientation_inverse_v, cross(robot_orientation_inverse_v, direction_vector));


			//cout << "direction_robot_frame: " << direction_robot_frame << endl;


			reachable = isReachable(direction_robot_frame);
			cout << "Reachable: " << reachable << endl;

			if(!reachable){
				cout<< "not reachable" << endl;
				//find which sub goal is closest to goal.
				// if it is reachable choose that sub goal; otherwise check the next one. 
				//hmm what is the best way to do this...

				//Start with transforming the sub goals to cartesian coordinates... 
				//or wait, we can also just transform the goal point to spherical coordinates.. 
				//or just neither dumbass..

				Vector3d subgoal_global_frame, subgoal_robot_frame, robot_orientation_v;
				float robot_orientation_w;

				robot_orientation_v [0] = robot_orientation[0];
				robot_orientation_v [1] = robot_orientation[1];
				robot_orientation_v [2] = robot_orientation[2];
				robot_orientation_w = robot_orientation[3];



				// anyways
				countx = 0;
				for (int m = 0; m < spherical_matrix_height; m++){
					for (int n = 0; n < spherical_matrix_width; n++){
						if(subgoal_matrix[m][n] > 0){
							vector <double> row;
							

							/*theta = ((m - 29) *3 - 1.5)*pi/180;
							phi = ((n-59)*3 - 1.5)*pi/180;
							x1 = subgoal_matrix[m][n]*sin(phi)*cos(theta);
							y1= subgoal_matrix[m][n]*sin(phi)*sin(theta);
							z1= subgoal_matrix[m][n]*cos(phi);*/


							//NEED to account for orientation of robot w.r.t. global frame..... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

							// Also, currently we are putting direction of goal in global frame into isreachable function
							// but the direction of sub goals in the robot frame into that function.... !!! this we need to fix
							rho = subgoal_matrix[m][n];
							theta = n *  pi / 60 - pi;
							phi = m * pi / 60 - pi/2;

							r = rho * cos(phi);				
							x1 = r * cos(theta);
							y1 = r * sin(theta);
							z1 = rho * sin(phi);

							subgoal_robot_frame[0] = x1;
							subgoal_robot_frame[1] = y1;
							subgoal_robot_frame[2] = z1;
							
							// the subgoal is not truly in global frame since we also have to add the position of the robot
							subgoal_global_frame = subgoal_robot_frame + 2*robot_orientation_w*cross(robot_orientation_v, subgoal_robot_frame) + 
							2*cross(robot_orientation_v, cross(robot_orientation_v, subgoal_robot_frame));

							// store matrix value
							double distance_sq = pow(goal_position[0] - (subgoal_global_frame[0]+robot_position[0]),2) + 
							pow(goal_position[1] - (subgoal_global_frame[1]+robot_position[1]), 2) + 
							pow(goal_position[2] - (subgoal_global_frame[2]+robot_position[2]), 2);

							//now push back to vector.. 
							distance_index_vector.push_back(make_pair(distance_sq, countx));
							// push back the m and n values as well..
							//matrix_indices_vector.push_back(make_pair(m,n));
							
							row.push_back((double)m);
							row.push_back((double)n);
							row.push_back(x1);
							row.push_back(y1);
							row.push_back(z1);
							
							subgoal_xyz.push_back(row);
							countx++;
						}
					}
				}

				//cout << "here" << endl;

				// sort stuff 
				sort(distance_index_vector.begin(), distance_index_vector.end(), comparator());

				cout << "done sorting sub goals by distance to goal, 715. " ;

				// print shit to see if works:

				//for(distance_and_index p : distance_index_vector){
				for(vitr = distance_index_vector.begin(); vitr != distance_index_vector.end(); ++vitr ){
					cout << vitr->first << " " << vitr->second << " | ";
				}
				cout << endl;
				cout << "now going to find the closest sugoal that's reachable. 724 ";
				for(vector<int>::size_type vecitr = 0; vecitr != distance_index_vector.size(); vecitr++){
					// make vector to 
					subgoal_vector[0] = subgoal_xyz[distance_index_vector[vecitr].second][2];
					subgoal_vector[1] = subgoal_xyz[distance_index_vector[vecitr].second][3];
					subgoal_vector[2] = subgoal_xyz[distance_index_vector[vecitr].second][4];
					//if subgoal reachable:

					cout << "# iteration over subgoals: " << vecitr << "  " ;

					cout << "coordinates of subgoal: " << subgoal_vector << endl;

					if(vecitr == distance_index_vector.size()-1) cout  << "Last sub goal " ;
					if(isReachable(subgoal_vector)){
						// This subgoal is reachable and therefore the selected subgoal

						cout <<"subgoal at squared and normal distance: " << distance_index_vector[vecitr].first<< "  " 
						<< sqrt(distance_index_vector[vecitr].first) << " chosen" << endl;
						//DO STUFF HERE!!!!

						// maybe turn into point cloud for debug reasons...

						PointCloud::Ptr msg (new PointCloud);
						msg->header.frame_id = "base_link";
						msg->height = 1;
						msg->width = 1;
						
						x = subgoal_vector[0];
						y = subgoal_vector[1];
						z = subgoal_vector[2];
						msg->points.push_back (pcl::PointXYZ(x,y,z));

						// Convert to ROS data type
					  	sensor_msgs::PointCloud2 output;
					  	pcl::toROSMsg(*msg, output);
					  	// Publish the data
					  	pub_selected_subgoal.publish (output);

					  	//delete msg; 


						break;
					}

				}
				cout << endl;
			}
		} 	
		else reachable = false;


		//cout << "yolo" << endl;
		//now to keep the robot in same place but only turn it:

		//set_pose.pose.position =  input->pose.position;
		/*set_pose.pose.position.x = robot_position[0];
		set_pose.pose.position.y = robot_position[1];
		set_pose.pose.position.z = robot_position[2];*/


		//for the purposes of visualization test in rviz, remove later maybe:
		//if(robot_position[2]<0.4){//input->pose.position.z < 0.3){
		// to get rid of drift, set fixed position:
		set_pose.pose.position.x = 0;
		set_pose.pose.position.y = 0;
		set_pose.pose.position.z = 0.4;
		//}

		set_pose.pose.orientation.x = 0;
		set_pose.pose.orientation.y = 0;
		set_pose.pose.orientation.z = z;
		set_pose.pose.orientation.w = w;
	}

	pub_desired_position_.publish(set_pose);
	//cout << "direction_vector: " << direction_vector << endl;
}

void GoalDirection::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input) {

	robot_position[0] = input->pose.position.x;
	robot_position[1] = input->pose.position.y;
	robot_position[2] = input->pose.position.z;

	robot_orientation[0] = input->pose.orientation.x;
	robot_orientation[1] = input->pose.orientation.y;
	robot_orientation[2] = input->pose.orientation.z;
	robot_orientation[3] = input->pose.orientation.w;


}

/*
//member function implementation for a service callback function
bool ExampleRosClass::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}*/



void GoalDirection::cameraCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::fromROSMsg (*input, camera_cloud);
}


//void GoalDirection::isReachable(const geometry_msgs::Vector3 & direction){  //?????????? veit ekki alveg hvernig er best ad gera thetta... 
bool GoalDirection::isReachable(const Vector3d & direction){ //float direction[4]){
//geometry_msgs::Vector3 direction){

	//check if direction is reachable from current robot location...

	cout << "in isReachable. 830 " ;
	float length_of_direction_vector_sq  = pow(direction[0],2) + pow(direction[1],2) + pow(direction[2],2);

	//cout << "direction: " << direction << " length_of_direction_vector_sq: " << length_of_direction_vector_sq << endl;

	


	//cout << "length_of_direction_vector_sq: " << length_of_direction_vector_sq << endl;
	//cout << "direction_vector: " << direction[0] << " " << direction[1] <<" " << direction[2] << endl;
	//----------------------------Iterate over all points in camera cloud-------------------------------------
	/*double x;
	double y;
	double z;*/
	float x,y,z;

	float iswithin;
	//double r, rho, phi, theta;
	//double pi = 3.14159265359;

	//For some reason, for the camera in the camera_link frame, z is forward, y down and x to the right. 
	// weird.

	vector <vector <double> > points_within_matr;

	vector <vector <double> > obstacle_points_direction_frame;
//for
//{
//} 
	//double sumx, sumy, sumz = 0.0;
	//int count = 0 ;

	// ------------------------------Let's make a discretized C space for using an A* like algorithm on--------------------

	cout << "make discretized C space, 865. ";
	int cspace_length = ceil(sqrt(length_of_direction_vector_sq) / cspace_resolution);

	int Cspace [cspace_length] [cspace_width] [cspace_width]  = {0};

	for(int i = 0; i < cspace_width; i++){
		for( int j = i ; j< cspace_width; j++){
			if(Cspaceframe[i][j] == 1){
				for ( int k = 0; k<cspace_length ; k++){
					Cspace[k][i][j] = 1;
					Cspace[k][j][i] = 1;
				}
			}
			else{
				for ( int k = 0; k<cspace_length ; k++){
					Cspace[k][i][j] = 0;
					Cspace[k][j][i] = 0;
				}
			}
		}
	}

	// find the edges of the Cspace that should be inaccessible:



	// now the forward axis should be in the middle of the Cspace, i.e. Cspace[cspace_width/2] [cspace_width/2][x]


	// ok now we should perhaps iterate through the stack of obstacle points... 



	// now, we want find the location of the obstacle points with respect to the direction vector 
	// so we need to find some the angle of rotation in xy plane and in the xz plane

	double xydistance = sqrt(pow(direction[0], 2) + pow(direction[1], 2));
	double xysintheta = direction[1] / xydistance;
	double xycostheta = direction[0] / xydistance;

	double xzdistance = sqrt(length_of_direction_vector_sq);
	double xzsintheta = direction[2] / xzdistance;
	double xzcostheta = xydistance / xzdistance;

	double new_y, new_x, new_z, newer_x;

	int obstacle_point_center_x, obstacle_point_center_y, obstacle_point_center_z;
	int corner_point_y, corner_point_z;

	int count_points_within = 0;
	cout << "Loop through points in camera point cloud, 915: " << endl;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = camera_cloud.begin(); it != camera_cloud.end(); it++){
		//cout << "in loop" << endl;

		if(!isnan(it->y)){
			//Since z is forward, y down and x to the right:
			// and we want x forward, y left and z up:
			y = -(it->x);
			z = -it->y + 0.13;
			x = it->z + 0.05;
			
			
			/*
			sumx += x;
			sumy += y;
			sumz += z;
			count++;
*/
			//cout << ".";

			//cout << "before cylinder function"<< endl;
			// check if point is within cylinder:
			//iswithin = CylTest_CapsFirst(direction, length_of_direction_vector_sq, radius_sq, x,y,z);

			// for finding all obstacle points within diameter distance:
			iswithin = CylTest_CapsFirst(direction, length_of_direction_vector_sq, diameter_sq, x,y,z);

			//cout << "after cylinder function"<< endl;

			if(iswithin >= 0){ //add to list... ;
				cout << "point within. 949 ";

				count_points_within++;

				vector <double> newColumn;
				newColumn.push_back(x);
				newColumn.push_back(y);
				newColumn.push_back(z);
				points_within_matr.push_back(newColumn);

				// Now, if we are going to work with C space, we should find the location of the obstacle points
				// with respect to the direction vector. i.e. we should map them as if the direction vector was the x (forward) axis.

				//so we first account for rotation in xy plane:
				new_x = x * xycostheta + y * xysintheta;
				new_y = -x * xysintheta + y * xycostheta;
				//then account for rotation in the xz plane: 
				newer_x = new_x * xzcostheta + z * xzsintheta;
				new_z = - new_x * xzsintheta + z * xzcostheta;

				vector <double> newColumn2;
				newColumn2.push_back(newer_x);
				newColumn2.push_back(new_y);
				newColumn2.push_back(new_z);
				obstacle_points_direction_frame.push_back(newColumn2);

				// ----------------------------------Now put expanded obstacle points into the Cspace-------------------------------
				//This code is very long for the sake of making it a little faster, i.e. checking some conditions before we go into the loops
				// find the centerpoint to be used for sphere w.r.t. the Cspace matrix.

				obstacle_point_center_x = floor(newer_x / cspace_resolution + 0.5);
				// since y is left.. and we want it right we put minus
				obstacle_point_center_y = - floor(new_y / cspace_resolution + 0.5) + cspace_half_width;
				obstacle_point_center_z = floor(new_z / cspace_resolution + 0.5) + cspace_half_width;
				//Now, we only want to fill in the points that are actually a part of the matrix...
				// so

				//addObstacleSphere(obstacle_point_center_x, obstacle_point_center_y, obstacle_point_center_z, cspace_length);

				if(obstacle_point_center_y < 0){

					// This means the center is to the left of the matrix
					// so we only have to fill up the right part of the sphere
					if (obstacle_point_center_z < 0){
						// Obstacle point below matrix
						// so only fill up upper right half of sphere
						cout << "point below left 995 ";

						// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
						// So the point we want to get to is:

						//technically we are dealing with a corner line, but it is a point in the yz plane
						corner_point_z = obstacle_point_center_z + cspace_half_width;
						corner_point_y = obstacle_point_center_y + cspace_half_width;

						// I guess we also have to check if the sphere goes out of bounds on either side length wise 

						//we can then iterate from the bottom left half of the matrix... 		
						/*
						Ah ya know, fuck this, let's just have an if statement that checks every time for now.. 
						if(obstacle_point_center_x - cspace_half_width< 0){
							// now we have to watch out so we don't get out of bounds :>P
							for(int y_itr = 0; y_itr < corner_point_y; y_itr++){
								for(int z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
									// we also have to iterate over this in x direction.. 
									count = 0;
									for(int x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ; x_itr++){
										if(sphere_model[count][-obstacle_point_center_y+y_itr][-obstacle_point_center_z+z_itr] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_x- count >=0){
												Cspace[obstacle_point_center_x- count][y_itr][z_itr]=1;
											}
										}
									count++;
									}
								}
							}
						}
						*/
						for(int y_itr = 0; y_itr < corner_point_y; y_itr++){
							for(int z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
								// we also have to iterate over this in x direction.. 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][-obstacle_point_center_y+y_itr][-obstacle_point_center_z+z_itr] == 1){
										if(x_itr < cspace_length) Cspace[x_itr][y_itr][z_itr] = 1;
										if(obstacle_point_center_x- countx >0) Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
									}
								countx++;
								}
							}
						}
					}
					else if (obstacle_point_center_z > cspace_width){
						cout << "point above left 1043 ";
						// Obstacle point above matrix
						// only fill in bottom right half of sphere

						// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
						// So the point we want to get to is:

						corner_point_z = obstacle_point_center_z - cspace_half_width;
						corner_point_y = obstacle_point_center_y + cspace_half_width;

						for(int y_itr = 0; y_itr < corner_point_y; y_itr++){
							countz =0;
							for(int z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
								// we also have to iterate over this in x direction.. 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][-obstacle_point_center_y+y_itr][obstacle_point_center_z-cspace_width+countz] == 1){
										if(x_itr < cspace_length) Cspace[x_itr][y_itr][z_itr] = 1;
										if(obstacle_point_center_x- countx >0) Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
									}
								countx++;
								}
							countz++;
							}
						}
					}
					else{
						// fill in the whole right half 
						// Athugum hvort vid seum fyrir ofan eda nedan midju:
						cout << "point left 1072 ";
						if(obstacle_point_center_z< cspace_half_width){
							// this means we are below middle
							// so we need to continuously check if the lower part of our sphere is out of bounds

							//technically we are dealing with a corner line, but it is a point in the yz plane
							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for(int y_itr = 0; y_itr < corner_point_y; y_itr++){
								countz = 0;
								for(int z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
									countx = 0;
									for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){
											if(x_itr < cspace_length){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
											} 
											if(obstacle_point_center_x- countx >0){
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z - countz >= 0){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z - countz]=1;
												}
											} 
										}
										countx++;
									}
									countz++;
								}
							}
						}
						else{
							// this means we are above middle
							// so we need to continuously monitor that the upper part of our sphere is not out of bounds

							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for(int y_itr = 0; y_itr < corner_point_y; y_itr++){
								countz =0;
								for(int z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){
											if(x_itr < cspace_length){ 
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
											}
											if(obstacle_point_center_x- countx >0){ 
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z + countz < cspace_width){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z + countz] = 1;
												} 
											}											
										}
									countx++;
									}
								countz++;
								}
							}
						}
					}
				}
				else if (obstacle_point_center_y > cspace_width){
					// This means the obstacle point is to the right of the matrix
					// so we only have to fill up the left part of the sphere
					cout <<"point right 1140 ";
					if (obstacle_point_center_z < 0){
						// Obstacle point below matrix
						// so only fill up upper left half of sphere
						cout << "below ";
						corner_point_z = obstacle_point_center_z + cspace_half_width;
						corner_point_y = obstacle_point_center_y - cspace_half_width;

						county = 0;
						for(int y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
							for(int z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
								// we also have to iterate over this in x direction.. 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][-obstacle_point_center_z+z_itr] == 1){
										if(x_itr < cspace_length) Cspace[x_itr][y_itr][z_itr] = 1;
										if(obstacle_point_center_x- countx >0) Cspace[obstacle_point_center_x - countx][y_itr][z_itr]=1;
									}
								countx++;
								}
							}
							county++;
						}
					}
					else if (obstacle_point_center_z > cspace_width){
						cout << "above " ;
						// Obstacle point above matrix
						// only fill in bottom left half of sphere
						corner_point_z = obstacle_point_center_z - cspace_half_width;
						corner_point_y = obstacle_point_center_y - cspace_half_width;
						county = 0;
						for(int y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
							countz =0;
							for(int z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
								// we also have to iterate over this in x direction.. 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][obstacle_point_center_z-cspace_width+countz] == 1){
										if(x_itr < cspace_length) Cspace[x_itr][y_itr][z_itr] = 1;
										if(obstacle_point_center_x- countx >0) Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
									}
									countx++;
								}
								countz++;
							}
							county++;
						}
					}
					else{
						// fill in the whole left half
						// Athugum hvort vid seum fyrir ofan eda nedan midju:

						if(obstacle_point_center_z< cspace_half_width ){
							// this means we are below middle
							// so we need to continuously check if the lower part of our sphere is out of bounds
							cout << "below middle ";
							//technically we are dealing with a corner line, but it is a point in the yz plane
							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;
							county = 0;
							for(int y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								countz = 0;
								for(int z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
									countx = 0;
									for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){
											if(x_itr < cspace_length){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
											} 
											if(obstacle_point_center_x- countx >0){
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z - countz >= 0){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z - countz]=1;
												}
											} 
										}
										countx++;
									}
									countz++;
								}
								county++;
							}
						}
						else{
							// this means we are above middle
							// so we need to continuously monitor that the upper part of our sphere is not out of bounds
							cout << "above middle ";
							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;
							county = 0;
							for(int y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								countz =0;
								for(int z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){
											if(x_itr < cspace_length){ 
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
											}
											if(obstacle_point_center_x- countx >0){ 
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z + countz < cspace_width){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z + countz] = 1;
												} 
											}											
										}
										countx++;
									}
									countz++;
								}
								county++;
							}
						}
					}
				}
				else if (obstacle_point_center_z < 0){
					// Obstacle point below matrix
					// fill up whole upper half of sphere
					cout  << "point below 1261 ";

					//well now, this is getting (unnecessary) lenghty, for only a little bit faster program... 
					// Let's check if we are to the right or left of the center:
					if(obstacle_point_center_y< cspace_half_width){
						// this means we are left of center, below matrix
						// need to monitor that left part of sphere does not go out of bounds

						cout << "left of center ";
						corner_point_z = obstacle_point_center_z + cspace_half_width;
						corner_point_y = obstacle_point_center_y + cspace_half_width;

						county = 0;
						for(int y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
							for(int z_itr = 0; z_itr < corner_point_z; z_itr ++ ){ 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
										if(x_itr < cspace_length){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
										} 
										if(obstacle_point_center_x- countx >0){
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y - county >= 0){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y - county][z_itr]=1;
											}
										} 
									}
									countx++;
								}
							}
							county++;
						}
					}
					else{
						// this means we are right of center, below matrix
						// need to monitor that right part of sphere does not go out of bounds

						cout <<"right of center " ;
						corner_point_z = obstacle_point_center_z + cspace_half_width;
						corner_point_y = obstacle_point_center_y - cspace_half_width;

						county = 0;
						for(int y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
							for(int z_itr = 0; z_itr < corner_point_z; z_itr ++){ 
								countx = 0;
								for(int x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
										if(x_itr < cspace_length){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;
										} 
										if(obstacle_point_center_x- countx >0){
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y + county < cspace_width){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y + county][z_itr]=1;
											}
										} 
									}
									countx++;
								}
							}
							county++;
						}
					}
				}
				else if (obstacle_point_center_z > cspace_width){
					// Obstacle point above matrix
					// fill whole bottom half of sphere

					cout << "point above 1332 ";

					//ugh, let's check if we are to right or left of middle point.. 
					if(obstacle_point_center_y< cspace_half_width){
						cout << "left of center " ;
						// this means we are left of center
						// need to monitor that left part of sphere does not go out of bounds
						corner_point_z = obstacle_point_center_z - cspace_half_width;
						corner_point_y = obstacle_point_center_y + cspace_half_width;

						county = 0;
						for(int y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
							countz = 0;
							for(int z_itr = cspace_width-1; z_itr > corner_point_z; z_itr -- ){ 
								countx = 0;
								for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][county][countz] == 1){
										if(x_itr < cspace_length){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
										} 
										if(obstacle_point_center_x- countx >0){
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y - county >= 0){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y - county][z_itr]=1;
											}
										} 
									}
									countx++;
								}
								countz++;
							}
							county++;
						}
					}
					else{
						// this means we are right of center
						// need to monitor that right part of sphere does not go out of bounds
						cout  << " right of center ";
						corner_point_z = obstacle_point_center_z - cspace_half_width;
						corner_point_y = obstacle_point_center_y - cspace_half_width;

						county = 0;
						for(int y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
							countz = 0;
							for(int z_itr = cspace_width-1; z_itr > corner_point_z; z_itr--){ 
								countx = 0;
								for(int x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
									if(sphere_model[countx][county][countz] == 1){
										if(x_itr < cspace_length){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;
										} 
										if(obstacle_point_center_x- countx >0){
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y + county < cspace_width){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y + county][z_itr]=1;
											}
										} 
									}
									countx++;
								}
								countz++;
							}
							county++;
						}
					}


				}
				else{
					// Fill in the whole sphere

					cout << "point middle 1405 ";
					// gerum bara einfaldan koda fyrir thetta til ad byrja med, er ordinn ansi threyttur a thessu...

					for(county = 0; county< cspace_half_width ; county ++){
						for(county = 0; county< cspace_half_width ; county ++){
							for(countx = 0; countx< cspace_half_width ; countx ++){
								if(sphere_model[countx][county][countz] == 1){

									if(obstacle_point_center_x+countx<cspace_length){
										if(obstacle_point_center_y+county<cspace_width){
											if(obstacle_point_center_z+countz<cspace_width){
												Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
											}
											if(obstacle_point_center_z-countz >=0){
												Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
											}
										}
										if(obstacle_point_center_y-county >=0){
											if(obstacle_point_center_z+countz<cspace_width){
												Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
											}
											if(obstacle_point_center_z-countz >=0){
												Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
											}
										}

									}
									if(obstacle_point_center_x- countx>0){
										if(obstacle_point_center_y+county<cspace_width){
											if(obstacle_point_center_z+countz<cspace_width){
												Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
											}
											if(obstacle_point_center_z-countz >=0){
												Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
											}
										}
										if(obstacle_point_center_y-county >=0){
											if(obstacle_point_center_z+countz<cspace_width){
												Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
											}
											if(obstacle_point_center_z-countz >=0){
												Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
											}
										}

									}


								}
							}
						}
					}
				} //her endar thessi langa ef setning 
				cout << "buid 1458. ";

			}// her 
		}
	}// her endar itrunin yfir myndavela punktana...

	//cout << "count_points_within : " << count_points_within << endl;

		//-------------------Transform again to point cloud for visualizing in rviz:------------------------
	cout << "Done creating Cspace, transform to point cloud: 1448 ";
	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "base_link";
	msg->height = 1;
	msg->width = cspace_length*Cspaceframe_activepoints;//cspace_width*cspace_width;


	/*cout<< "msg width: " << msg->width <<endl;
	cout<< "cspace_length: " << cspace_length << endl;
	cout << "Cspaceframe_activepoints" << Cspaceframe_activepoints << endl;*/
	//cout << "cspace_width: "<< cspace_width<< endl;
	//cout << "Cspace length: " <<  sizeof Cspace / sizeof Cspace[0] << endl; // 2 rows  
	//cout << "cspacesomething : " << sizeof Cspace[0]/ sizeof(int) << endl;
  	//int cols = sizeof Cspace[0] / sizeof(0); // 5 cols
	
	countx =0;
	for (int y_itr=0; y_itr< cspace_width; y_itr++){
		for (int z_itr=0;z_itr<cspace_width; z_itr++){
			if(Cspaceframe[y_itr][z_itr] == 0){
				for (int x_itr= 0; x_itr<cspace_length; x_itr++){
					if(Cspace[x_itr][y_itr][z_itr] == 0){
						msg->points.push_back (pcl::PointXYZ(0,0,0));
					}
					else{//if(Cspace[x_itr][y_itr][z_itr] != 0){
						// minus y because before we changed the definition from pointing to left to pointing to right... !!!!!!!!!
						msg->points.push_back (pcl::PointXYZ(x_itr*cspace_resolution,-y_itr*cspace_resolution,z_itr*cspace_resolution));
					}
					//cout <<" x:" <<x_itr << " y:" <<y_itr<<" z:" << z_itr ;
					countx ++;
				}
			}				
		}
	}
	
	//cout << "cloud size: " << countx << endl;
	//cout << "whats up" << endl;
	// Convert to ROS data type
  	sensor_msgs::PointCloud2 output;
  	pcl::toROSMsg(*msg, output);
  	// Publish the data
  	pub_Cspace.publish (output);

  	//cout << "done with filling occupancy cloud" << endl;

  	//------------------------------------Now use graph search to find if subgoal is reachable:--------------------------------------

  	cout << "now use graph search, 1494: ";
  	if (count_points_within == 0){
  		// this means completely unblocked way to goal 
  		return true;
  	}
  	else {
	  	int edge_points [cspace_length][cspace_width][cspace_width] = {0};
	  	//int reachable_points [cspace_length][cspace_width][cspace_width] = {0};

	  	 //Let's try using the Cspace matrix as reachable points matrix
	  	int  front_edge = 0;//, next_back_edge = 0;
	  	edge_points[0][cspace_half_width][cspace_half_width] = 1;
	  	Cspace[0][cspace_half_width][cspace_half_width] = 2;
		// Now 0 in Cspace means it is free, 1 means it is occupied,  2 means it is reachable...

	  	//Traverse through matrix and make sure to check where back and front edge are:
	  	bool condition = true;
	  	//bool backwards_mode = false;
	  	int new_front_edge;
	  	while(condition){
	  		new_front_edge = -1;
	  		//cout << "in while loop" << endl;
	  		//cout << "front edge: " << front_edge << endl;
		  	for (int y_itr = 0; y_itr < cspace_width; y_itr++){
		  		for (int z_itr=0; z_itr<cspace_width; z_itr++){
		  			// check if point is in edge points:
		  			if(edge_points[front_edge][y_itr][z_itr] == 1){
		  				//cout << "found edge point" << endl;
		  				// mark all surrounding points that are free as edge points 
		  				if(y_itr < cspace_width -1){
			  				if(Cspace[front_edge][y_itr+1][z_itr] == 0){
			  					Cspace[front_edge][y_itr+1][z_itr] = 2;
			  					edge_points[front_edge][y_itr+1][z_itr] = 1;
			  					if(new_front_edge < front_edge) new_front_edge = front_edge;
			  				}
			  			}
			  			//cout << "2";
			  			if(y_itr>0){
			  				if(Cspace[front_edge][y_itr-1][z_itr] == 0){
			  					Cspace[front_edge][y_itr-1][z_itr] = 2;
			  					edge_points[front_edge][y_itr-1][z_itr] = 1;
			  					if(new_front_edge < front_edge) new_front_edge = front_edge;
			  				}
			  			}
			  			//cout << "3";
			  			if(z_itr< cspace_width -1){
			  				if(Cspace[front_edge][y_itr][z_itr+1] == 0){
			  					Cspace[front_edge][y_itr][z_itr+1] = 2;
			  					edge_points[front_edge][y_itr][z_itr+1] = 1;
			  					if(new_front_edge < front_edge) new_front_edge = front_edge;
			  				}
			  			}
			  			//cout << "4";
			  			if(z_itr >0) {
			  				if(Cspace[front_edge][y_itr][z_itr-1] == 0){
			  					Cspace[front_edge][y_itr][z_itr-1] = 2;
			  					edge_points[front_edge][y_itr][z_itr-1] = 1;
			  					if(new_front_edge < front_edge) new_front_edge = front_edge;
			  				}
			  			}
			  			//cout << "5";
			  			if(Cspace[front_edge+1][y_itr][z_itr] == 0){
		  					Cspace[front_edge+1][y_itr][z_itr] = 2;
		  					edge_points[front_edge+1][y_itr][z_itr] = 1;
		  					new_front_edge = front_edge+1;
		  					goto next;
		  				}
			  			//cout << "6" ; 
		  				// unmark this point as edge point
		  				edge_points[front_edge][y_itr][z_itr] = 0;
		  				// check if next points should be marked as front edge (or back edge...)
		  			}
		  		}
		  	}
		  	next:
		  	//cout<< "new_front_edge: " << new_front_edge << endl;
		  	if(new_front_edge == -1){
		  		// Now we need to move backwards; count first if all points in cross section are reachable:
		  		countx = 0;
		  		for (int y_itr = 0; y_itr < cspace_width; y_itr++){
		  			for (int z_itr=0; z_itr<cspace_width; z_itr++){
		  				if(Cspace[front_edge][y_itr][z_itr] == 2) countx += 1;
		  			}
		  		}
		  		if(countx == Cspaceframe_activepoints){
		  			// this means we are going backwards but all points in cross section are reachable.. 
		  			cout << "not reachable" << endl;
		  			return false;
		  			break;
		  			//condition =false;
		  		} 
		  		front_edge -= 1;
		  	}
		  	else{
		  		front_edge = new_front_edge;
		  	}
		  	if(front_edge == cspace_length - 1){
		  		//now we have reached our goal..
		  		// but need to check if whole section is free..

		  		int count_new_points = 1;
		  		// We want to fill the last cross section...
		  		while(count_new_points !=0){
		  			count_new_points = 0;
			  		for (int y_itr = 0; y_itr < cspace_width; y_itr++){
				  		for (int z_itr=0; z_itr<cspace_width; z_itr++){
				  			// check if point is in edge points:
				  			if(edge_points[front_edge][y_itr][z_itr] == 1){
				  				//cout << "found edge point" << endl;
				  				// mark all surrounding points that are free as edge points 
				  				if(y_itr < cspace_width -1){
					  				if(Cspace[front_edge][y_itr+1][z_itr] == 0){
					  					Cspace[front_edge][y_itr+1][z_itr] = 2;
					  					edge_points[front_edge][y_itr+1][z_itr] = 1;
					  					count_new_points++;
					  				}
					  			}
					  			//cout << "2";
					  			if(y_itr>0){
					  				if(Cspace[front_edge][y_itr-1][z_itr] == 0){
					  					Cspace[front_edge][y_itr-1][z_itr] = 2;
					  					edge_points[front_edge][y_itr-1][z_itr] = 1;
					  					count_new_points++;
					  				}
					  			}
					  			//cout << "3";
					  			if(z_itr< cspace_width -1){
					  				if(Cspace[front_edge][y_itr][z_itr+1] == 0){
					  					Cspace[front_edge][y_itr][z_itr+1] = 2;
					  					edge_points[front_edge][y_itr][z_itr+1] = 1;
					  					count_new_points++;
					  				}
					  			}
					  			//cout << "4";
					  			if(z_itr >0) {
					  				if(Cspace[front_edge][y_itr][z_itr-1] == 0){
					  					Cspace[front_edge][y_itr][z_itr-1] = 2;
					  					edge_points[front_edge][y_itr][z_itr-1] = 1;
					  					count_new_points++;
					  				}
					  			}
					  			//cout << "6" ; 
				  				// unmark this point as edge point
				  				edge_points[front_edge][y_itr][z_itr] = 0;
				  				// check if next points should be marked as front edge (or back edge...)
				  			}
				  		}
				  	}
			    }

			    //Now let's check if the middle point of the last cross section is free
			    //if(Cspace[front_edge][cspace_half_width][cspace_half_width] == 2){
			    // no, let's check the total number of points instead, want it to be completely free maybe...
			    countx = 0;
			    for (int y_itr = 0; y_itr < cspace_width; y_itr++){
				  	for (int z_itr=0; z_itr<cspace_width; z_itr++){
				  		// check if point is in edge points:
				  		//if(edge_points[front_edge][y_itr][z_itr] == 1){
				  		if(Cspace[front_edge][y_itr][z_itr] != 1){ // means C space is not occupied in this point.. 
				  				countx++;
						}
					}
				}
				cout << "count of points: "<< countx <<  endl;
			    if(countx == Cspaceframe_activepoints){
			    	cout << "subgoal reachable " ;
			    	return true;
			    }
		  		else{
		  			cout << "subgoal not reachable" << endl;
		  			return false;	
		  		} 
		  		cout << "now break, 1665. ";
		  		break;
		  		condition =false;
		  	}
	  	}
	}

}



//-----------------------------------------Function that tests if point lies within cylinder in 3D:-----------------------------------
//taken from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
/*struct Vec3
{
	float x;
	float y;
	float z;
};*/
// use Vector3 instead of Vec3:

float GoalDirection::CylTest_CapsFirst(const Vector3d & dir_vec, float lengthsq, float  radius_sq, float  pdx, float pdy, float pdz)
	//float dir_vec [4], float lengthsq, float  radius_sq, float  pdx, float pdy, float pdz)
//Vector3d & dir_vec, float lengthsq, float  radius_sq, float  pdx, float pdy, float pdz)
//float & lengthsq, float & radius_sq, float & pdx, float & pdy, float & pdz)
//geometry_msgs::Vector3 & dir_vec, float & lengthsq, float & radius_sq, float & pdx, float & pdy, float & pdz) 
//const Vec3 & pt1, const Vec3 & pt2, float lengthsq, float radius_sq, const Vec3 & testpt )
{
	float dx, dy, dz;	// vector d  from line segment point 1 to point 2
	//float pdx, pdy, pdz;	// vector pd from point 1 to test point
	float dot, dsq;

	dx = dir_vec[0];//.x; //pt2.x - pt1.x;	// translate so pt1 is origin.  Make vector from
	dy = dir_vec[1];//.y; //pt2.y - pt1.y;     // pt1 to pt2.  Need for this is easily eliminated
	dz = dir_vec[2];//.z; //pt2.z - pt1.z;

/*
	pdx = testpt.x ;//- pt1.x;		// vector from pt1 to test point.
	pdy = testpt.y ;//- pt1.y;
	pdz = testpt.z ;//- pt1.z;
*/
	// Dot the d and pd vectors to see if point lies behind the 
	// cylinder cap at pt1.x, pt1.y, pt1.z

	dot = pdx * dx + pdy * dy + pdz * dz;

	// If dot is less than zero the point is behind the pt1 cap.
	// If greater than the cylinder axis line segment length squared
	// then the point is outside the other end cap at pt2.

	if( dot < 0.0f || dot > lengthsq )
	{
		return( -1.0f );
	}
	else 
	{
		// Point lies within the parallel caps, so find
		// distance squared from point to line, using the fact that sin^2 + cos^2 = 1
		// the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
		// Carefull: '*' means mult for scalars and dotproduct for vectors
		// In short, where dist is pt distance to cyl axis: 
		// dist = sin( pd to d ) * |pd|
		// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
		// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
		// dsq = pd * pd - dot * dot / lengthsq
		//  where lengthsq is d*d or |d|^2 that is passed into this function 

		// distance squared to the cylinder axis:

		dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

		if( dsq > radius_sq )
		{
			return( -1.0f );
		}
		else
		{
			//cout << "obstacle point" <<  endl;
			return( dsq );		// return distance squared to axis
		}
	}
}



Vector3d GoalDirection::cross(const Vector3d & vec1, const Vector3d & vec2){
	Vector3d cross_product;
	cross_product[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
	cross_product[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
	cross_product[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
	
	return ( cross_product );
}

int main(int argc, char** argv)
{

// ROS set-ups:
    ros::init(argc, argv, "goal_direction"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type GoalDirection");
    GoalDirection goalDirection(&nh);  //instantiate an GoalDirection object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");

    //Wait a second, maybe do all the work here instead... 

    ros::spin();
	return 0;
}