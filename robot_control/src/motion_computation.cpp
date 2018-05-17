// template from:      https://github.com/wsnewman/learning_ros/tree/master/Part_1/example_ros_class/src

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "motion_computation.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
MotionComputation::MotionComputation(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of MotionComputation");
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
    safety_distance = 0.4;
    max_sonar_range = 6;
    radius_sq = pow(robot_radius,2);
    diameter_sq = pow(robot_radius*2,2);

    spherical_matrix_degree_resolution = 6;
    spherical_matrix_height = 180/spherical_matrix_degree_resolution;
    spherical_matrix_width = 360/spherical_matrix_degree_resolution;
	M = spherical_matrix_height;//180/spherical_matrix_degree_resolution;
    N = spherical_matrix_width;//360/spherical_matrix_degree_resolution;

    subgoal_point = false;
    Cspaceframe_activepoints = 0;
    goal_point = false;

    cspace_resolution = 0.05; // meters
    cspace_width = ceil(robot_diameter / cspace_resolution);
    cspace_half_width = ceil(robot_radius / cspace_resolution);
	n_radius = robot_radius/ cspace_resolution;

    //robot_radius / cspace_resolution;
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
    initializeUnitaryVectors();


    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void MotionComputation::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    // add more subscribers here, as needed
    //camera_subscriber_ = nh_.subscribe("/camera/depth/points", 1, &MotionComputation::cameraCallback, this); 
    goal_position_sub_ = nh_.subscribe("/goal_position", 1, &MotionComputation::goalPositionCallback, this);
	occupancy_matrix_sub_ = nh_.subscribe("/spherical_matrix", 1, &MotionComputation::sphericalMatrixCallback, this);
	//sonar_limits_and_ranges_sub_ = nh_.subscribe("/sonar_degree_limits", 1, &MotionComputation::sonarlimitsrangesCallback, this);
	sonar_up_subscriber_ = nh_.subscribe("/sonar_up/range", 1, &MotionComputation::sonarupCallback, this);    //<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
    sonar_down_subscriber_ = nh_.subscribe("/sonar_down/range", 1, &MotionComputation::sonardownCallback, this);  
	//camera_subscriber_ = nh_.subscribe("/voxelpoints", 1, &MotionComputation::cameraCallback, this);
	robot_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &MotionComputation::robotPositionCallback, this);
	octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &MotionComputation::octoMapCallback, this);
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

void MotionComputation::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 
    pub = nh_.advertise<sensor_msgs::PointCloud2>("spherical_cloud", 1, true);
    pub_Cspace = nh_.advertise<sensor_msgs::PointCloud2>("Cspace_cloud", 1, true);
    pub_subgoal_cloud= nh_.advertise<sensor_msgs::PointCloud2>("subgoal_cloud", 1, true);
    pub_selected_subgoal = nh_.advertise<sensor_msgs::PointCloud2>("selected_subgoal", 1, true);
    pub_u_dom_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/u_dom_cloud",1, true);
    pub_u_sol_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/u_sol_cloud", 1, true);

    

    //pub_desired_position_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1, true);
    //pub_target_vector = nh_.advertise<geometry_msgs::Vector3>("/target_vector",1, true);
    //pub_u_sol = nh_.advertise<geometry_msgs::Vector3>("/u_sol",1, true);
    pub_target_position = nh_.advertise<geometry_msgs::Vector3>("/target_position",1, true);


    
    //pub_u_targ = nh_.advertise<geometry_msgs::Vector3>("/u_targ",1, true);

}


void MotionComputation::initializeUnitaryVectors()
{
	double theta, phi, r;

	for(m = 0 ; m<M; m++){
		for(n=0;n<N; n++){
			theta = n *  pi / M - pi;
			phi = m * pi / M - pi/2;
			r = cos(phi);
			unitary_direction_vectors_matrix[m][n][0] = r * cos(theta);
			unitary_direction_vectors_matrix[m][n][1] = r * sin(theta);
			unitary_direction_vectors_matrix[m][n][2] = sin(phi);
		}
	}

	// write out unitary direction vectors matrix:
	/*cout << "unitary direction vector matrix: " << endl;
	for(int i=0; i<3; i++){
		cout << "vector component: " << i << endl;
		for(m = M-1 ; m >=0; m--){
			for(n = N-1; n >=0; n--){
				cout << unitary_direction_vectors_matrix[m][n][i] << " ";
			}
			cout << endl;
		}
	}*/

}


void MotionComputation::initializeSphere()
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
					/*if(i==0){
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
					}*/
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

						/*if(i==0){
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
						}*/
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

						/*if(i == 0){
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
						}*/
					}
				}
			}
			//start at 2cm (cspace resolution)
			//end at robot radius...
		}
		ivalue += cspace_resolution;
	}

	for( int j = 0 ; j <n_radius; j++){
		for(int k=j ; k<n_radius; k++){
			jvalue = cspace_resolution*(j+1);
			kvalue = cspace_resolution*(k+1);
			if( pow (jvalue, 2) + pow(kvalue,2) <= radius_sq){
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
			else{
				Cspaceframe[k + cspace_width/2][j + cspace_width/2] = 1;
				Cspaceframe[j + cspace_width/2][k + cspace_width/2] = 1;
				Cspaceframe[k + cspace_width/2][cspace_width/2-j-1] =1;
				Cspaceframe[cspace_width/2-j-1][k + cspace_width/2] =1;
				Cspaceframe[j + cspace_width/2][cspace_width/2-k-1]=1;
				Cspaceframe[cspace_width/2-k-1][j + cspace_width/2]=1;
				Cspaceframe[ cspace_width/2-k-1][ cspace_width/2-j-1] = 1;
				Cspaceframe[ cspace_width/2-j-1][ cspace_width/2-k-1] = 1;

			}

		}
	}

	// When active points is different:

        //let's write out the sphere model:
        /*
        for (int i = 0 ; i < n_radius ; i++){
                for ( int j = 0 ; j < n_radius ; j++){
                        for (int k = 0 ; k < n_radius; k ++){
                            cout << sphere_model[i][j][k];
                        }
                        cout << endl;
                }
                cout<< endl << endl;
        }*/

	//lets write out our frame:

	cout <<"Cspaceframe: " << endl;
	for(int i = 0; i < cspace_width ; i++){
		for (int j = 0; j<cspace_width; j++){
			cout<< Cspaceframe[i][j];
		}
		cout << endl;
	}
}

/*void MotionComputation::sonarlimitsrangesCallback(const std_msgs::Int16MultiArray::ConstPtr& input){
	sonar_up_limit = input -> data[0];
	range_up = input -> data[1];
	sonar_down_limit = input -> data[2];
	range_down = input -> data[3];
	//cout << "sonar up limit: " << sonar_up_limit << "sonar down limit: " 
	//<< sonar_down_limit << "range_up: " << range_up << "range_down: " << range_down << endl;
}*/
void MotionComputation::sonarupCallback(const sensor_msgs::RangeConstPtr& input) {
    range_up = input->range;
}
void MotionComputation::sonardownCallback(const sensor_msgs::RangeConstPtr& input) {
    range_down = input->range;
    //cout << "in sonar down callback, range down: " << range_down;
}


void MotionComputation::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){
//double [60][120] MotionComputation::sphericalMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& matrix_msg){

	//cout << "in sphericalMatrixCallback" <<  endl;

	float dstride0 = matrix_msg->layout.dim[0].stride;
	float dstride1 = matrix_msg->layout.dim[1].stride;
	/*float h = matrix_msg->layout.dim[0].size;
	float w = matrix_msg->layout.dim[1].size;
	int M = h;
	int N = w;*/

	/*
		float w = msg->layout.dim[1].size;
	// Below are a few basic Eigen demos:
	//std::vector<float> data = matrix_msg->data;
	//Eigen::Map<Eigen::MatrixXf> mat(data.data(), h, w);
*/
	//anyways,  let's try converting this to array:

	
	//double sphere_matrix [M][N] = {0}; defined in .h file
	for (m = 0; m< M ; m++){
		for (n = 0; n< N ; n++){
			sphere_matrix[m][n] = matrix_msg->data[m*dstride1 + n];
			subgoal_matrix[m][n] = 0;
			} 
	}

	
	/*cout << "sphere matrix: " << endl;// << sphere_matrix << endl;
	for (m = M-1; m>=0 ; m--){
		for(n=0;n<N;n++){
			cout << sphere_matrix[m][n] << " " ;
		}
		cout << endl;
	}*/	

	robot_sphere_matrix_position[0] = sphere_matrix[0][0];
	robot_sphere_matrix_position[1] = sphere_matrix[0][1];
	robot_sphere_matrix_position[2] = sphere_matrix[0][2];

	robot_sphere_matrix_orientation[0] = sphere_matrix[0][3];
	robot_sphere_matrix_orientation[1] = sphere_matrix[0][4];
	robot_sphere_matrix_orientation[2] = sphere_matrix[0][5];
	robot_sphere_matrix_orientation[3] = sphere_matrix[0][6];


	//-------------------Transform again to point cloud for visualizing in rviz:------------------------

	double x, y, z, r, rho, phi, theta;

	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "base_link";
	msg->height = 1;
	msg->width = M*N;

	  //for(int i = 0, max = 120*60; i!=max, i++) {
	for( m=0; m < M; m++){
		for(n=0; n<N; n++){
			if(sphere_matrix[m][n] ==0){
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



  	//here I'm assuming that I want to move the robot up/down by the distance of its diameter
  	// if that will be chosen as the most desireable direction...
  	//maybe this should be done in some more clever way
  	/*
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
  	//Add sub goals for up / down ...

  	// ---Here we only locate sub goals in the view of the camera and directly above and below the robot---
  	// Maybe they should be located in more places.. 

  	if(goal_point){
  		// depending on the goal point and the range measurement up / down:
  		double relative_height = goal_position[2] - robot_sphere_matrix_position[2];//robot_position[2];    //range_down
  		if(relative_height > 0){
  			//Locate sub goal above robot
  			if(relative_height < range_up +sonar_up_vertical_offset - robot_radius){
  				subgoal_matrix[M-1][0] = relative_height;
  			}
  			else{
  				// now the height is higher than we can clearly see with sonar
  				if(range_up+sonar_up_vertical_offset>robot_diameter) subgoal_matrix[M-1][0] = range_up + sonar_up_vertical_offset- robot_radius;
  			}

  			// Also locate some sub goal below robot ... 
  			if(range_down - sonar_down_vertical_offset - robot_diameter > 1){
  				subgoal_matrix[0][0] = 0.7;
  			}
  			else if(range_down - sonar_down_vertical_offset - robot_diameter > robot_diameter){
  				subgoal_matrix[0][0] = (range_down - sonar_down_vertical_offset)/2;
  			}
  		}
  		else{
  			// locate sub goal below robot
  			if(abs(relative_height) < range_down - sonar_down_vertical_offset - robot_radius){
  				subgoal_matrix[0][0] = - relative_height;
  			}
  			else{
  				// now the height is higher than we can clearly see with sonar
  				if(range_down-sonar_down_vertical_offset>robot_diameter) subgoal_matrix[0][0] = range_down - sonar_down_vertical_offset - robot_radius;
  			}

  			// Also locate some sub goal above robot ... 
  			if(range_up + sonar_up_vertical_offset - robot_diameter > 1){
  				subgoal_matrix[M-1][0] = 0.7;
  			}
  			else if(range_up + sonar_up_vertical_offset - robot_diameter > robot_diameter){
  				subgoal_matrix[M-1][0] = (range_up + sonar_up_vertical_offset)/2;
  			}
  		}
  	}

  	//Now let's add stuff for the camera view... :

  	// we only have to account for area of matrix that is in view of camera
  	//change this if FOV of camera changes !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  	//Brace yourselves, we will have many if statements

  	double difference;

  	//double statement;

  	//cout << "camera_x_offset + max_camera_range : " << camera_x_offset + max_camera_range <<  endl;

  	// Perhaps remove the following since we are not only thinking about what is in the camera frame...
  	// ok, have to rewrite most of this shit:

  	//Lets just check within vertical field of camera.. 
  	//int m_min = M * 28/60;
  	//int m_max = M * 40/60;
  	
  	//max is like 27-41 for M = 60
  	int m_min = M * 15/60;
  	int m_max = M * 45/60;


  	//cout << "m min " << m_min << " m max " << m_max << " M " << M << endl;
  	// or we could omit sub goals where there is unknown in the octomap.. 
	for( m = m_min ; m < m_max ; m++){  //max is like 27-41 for M = 60
		for( n = 0; n< N ; n++){ //and 50 - 68 for N = 120

			//To make sure we're not on the edge of view of the camera:
			//if(sphere_matrix[m][n] != 0){//robot_radius + safety_distance){

			//Check horizontal difference:
			difference = abs(sphere_matrix[m][n]-sphere_matrix[m][n+1]);
			if(difference > 2*robot_radius ){// - (robot_radius + safety_distance)) >0.001){ 
			//sphere_matrix[m][n+1] != robot_radius + safety_distance){
				// this means that there is enough difference between these points to count
				// as subgoal points...

				/*cout << "camera_x_offset + max_camera_range : " << camera_x_offset + max_camera_range <<  endl;
				cout << "sphere_matrix[m][n]: " << sphere_matrix[m][n] <<  endl;
				statement = sphere_matrix[m][n] - (camera_x_offset + max_camera_range);
				cout << "statement: " << statement <<  endl;*/
				if(sphere_matrix[m][n] ==0){  //just to get rid of some dumb ass floating point error
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
				else if(sphere_matrix[m][n+1] ==0){
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
			if(difference > 2*robot_radius){
				// this means that there is enough difference between these points to count
				// as subgoal points...

				if(sphere_matrix[m][n] ==0){
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
				else if(sphere_matrix[m+1][n] ==0){
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
			//}
		}
	}



	//-------------------Transform again to point cloud for visualizing in rviz:------------------------

	//double x, y, z, r, rho, phi, theta;

	PointCloud::Ptr subgoal_msg (new PointCloud);
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
  	pub_subgoal_cloud.publish (subgoal_output);

  	//cout << "done with runnig sphericalMatrixCallback"<< endl;

  	/*if(goal_point){
  		direction_vector = goal_position - robot_position;

	    //cout<<"here 2"<< endl;
	    //cout << "robot_vector: " << robot_position[0] << " " << robot_position[1] <<" " << robot_position[2] << endl;
	    //cout << "goal_position: " << goal_position[0] << " " << goal_position[1] <<" " << goal_position[2] << endl;

	    //cout << "direction_vector: " << direction_vector[0] << " " << direction_vector[1] <<" " << direction_vector[2] << endl;

	    xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));

	    set_pose.header.frame_id = "map";
	    set_pose.header.stamp = ros::Time::now();

        double theta = acos(direction_vector[0]/xy_length_of_direction_vector);
        if(direction_vector[1] < 0){
                theta = -theta;
        }

        //cout << "theta is: " << theta << endl;
        //and we want to turn around z axis so our quaternion coordinates become:
        double x= 0;
        double y = 0;
        z_turn = 1*sin(theta/2);
        w_turn = cos(theta/2);

        set_pose.pose.position.x = 0;
        set_pose.pose.position.y = 0;
        set_pose.pose.position.z = 0.4;
        //}

        set_pose.pose.orientation.x = 0;
        set_pose.pose.orientation.y = 0;
        set_pose.pose.orientation.z = z_turn;
        set_pose.pose.orientation.w = w_turn;


	    pub_desired_position_.publish(set_pose);
  	}*/

}


void MotionComputation::goalPositionCallback(const geometry_msgs::Vector3ConstPtr& input) {

	//cout << "in goal position callback " << endl;

	goal_position[0] = input->x;
	goal_position[1] = input->y;
	goal_position[2] = input->z;
/*
	goal_position.push_back(input->x);
	goal_position.push_back(input->y);
	goal_position.push_back(input->z);
	*/
	//cout<<"here 1"<< endl;

	direction_vector = goal_position - robot_sphere_matrix_position;//robot_position;
    goal_point = true;
    bool nothing_reachable = false;

    //geometry_msgs::PoseStamped set_pose;

    //cout<<"here 3"<< endl;


   vector<distance_and_index> distance_index_vector;
   vector<distance_and_index> :: iterator vitr;
   //vector<pair<int,int>> matrix_indices_vector;
   vector< vector<double> > subgoal_xyz; //indeces_xyz;
   double r, rho, phi, theta;

   double x1,y1,z1, x2, y2, z2;

	xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));
    //cout<<"what's happening"<< endl;
    //if the goal is not directly above or below the robot:
	target_calculated = false;
    if(abs(direction_vector[2])/ xy_length_of_direction_vector < 2.8){

        //turn robot in xy plane in direction of goal

        //find desired degrees in xy plane:

        //usually x forward, y left and z up:
        // so maybe do like this:?

        bool reachable;
        //cout << "theta " << theta << endl;
        //cout << "difference in orientation: " << abs(z - robot_orientation[2]) + abs(w - robot_orientation[3]) << endl;
        //cout << "desired quaternion coordinates: " << "x: " << x << "y: " <<y << "z: " << z <<"w: " <<w << endl;
        //cout << "actual quaternion coordinates: " << "x: " << robot_orientation[0] << "y: " <<robot_orientation[1] << "z: "
        //<< robot_orientation[2] <<"w: " <<robot_orientation[3] << endl;

        // only check if reachable if within camera limits..
        // vertical field of view of camera only 45 degrees so..
        //cout << "checking if goal within camera limits, line 600. ";
        
        double theta = atan2(direction_vector[1],direction_vector[0]);//acos(direction_vector[0]/xy_length_of_direction_vector);
        /*if(direction_vector[1] < 0){
                theta = -theta;
        }*/

        //cout << "theta is: " << theta << endl;
        //and we want to turn around z axis so our quaternion coordinates become:
        //z_turn = 1*sin(theta/2);
        //w_turn = cos(theta/2);

        //var yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
        
        /*
        heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
		attitude = asin(2*qx*qy + 2*qz*qw)
		bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

		except when qx*qy + qz*qw = 0.5 (north pole)
		which gives:
		heading = 2 * atan2(x,w)
		bank = 0
		and when qx*qy + qz*qw = -0.5 (south pole)
		which gives:
		heading = -2 * atan2(x,w)*/

        //double yaw = atan2(2.0*(robot_orientation[1]*robot_orientation[2] + robot_orientation[3]*robot_orientation[0]), robot_orientation[3]*robot_orientation[3] - robot_orientation[0]*robot_orientation[0] - robot_orientation[1]*robot_orientation[1] + robot_orientation[2]*robot_orientation[2]);
        /*double yaw = atan2(2*robot_orientation[1]*robot_orientation[3]-2*robot_orientation[0]*robot_orientation[2] , 1 - 2*pow(robot_orientation[1],2) - 2*pow(robot_orientation[2],2));
        double check_this = robot_orientation[0]*robot_orientation[1] + robot_orientation[2]*robot_orientation[3];
        if( check_this < 0.53 && check_this > 0.47) yaw = 2 * atan2(robot_orientation[0],robot_orientation[3]);
        else if(check_this > -0.53 && check_this < -0.47) yaw = -2 * atan2(robot_orientation[0],robot_orientation[3]);*/

        /*tf2::Quaternion q;
    	q[0] = robot_orientation[0];
    	q[1] = robot_orientation[1];
    	q[2] = robot_orientation[2];
    	q[3] = robot_orientation[3];
    	tf2::Matrix3x3 mat(q);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

        //double yaw = atan2(2.0*(robot_orientation[1]*robot_orientation[2] + robot_orientation[3]*robot_orientation[0]), robot_orientation[3]*robot_orientation[3] - robot_orientation[0]*robot_orientation[0] - 
        //	robot_orientation[1]*robot_orientation[1] + robot_orientation[2]*robot_orientation[2]);

        //cout <<" here " << endl;

        cout << "yaw: " << yaw << " theta: " << theta <<endl;*/

        /*if((abs(z_turn - robot_orientation[2]) + abs(w_turn - robot_orientation[3])  < 0.05 
        	|| abs(z_turn + robot_orientation[2]) + abs(w_turn + robot_orientation[3]) < 0.05)
                && abs(direction_vector[2])/ xy_length_of_direction_vector < 0.36 || subgoal_point){ */// for within 20 degrees... //< 0.6 || subgoal_point){
        //if((abs(yaw - theta) < 0.2)
        //         || subgoal_point){ //&& abs(direction_vector[2])/ xy_length_of_direction_vector < 0.36
	    


        //cout << "it is within limits. ";
        //cout << "going into isReachable function" << endl ;
        // To get the direction in the same frame as the robot we need to turn the direction vector
        // to do this we find the inverse of the robot quaternion and use it to turn the direction vector
        // why am I using the inverse? fml

        // inverse of robot quaternion:
        /*Vector3d robot_orientation_inverse_v;
        float robot_orientation_inverse_w;

        float robot_orientation_sum_squared = pow(robot_orientation[0],2) + pow(robot_orientation[1],2) + pow(robot_orientation[2],2)
        +pow(robot_orientation[3],2);

        robot_orientation_inverse_w = robot_orientation[3] / robot_orientation_sum_squared;

        robot_orientation_inverse_v[0] = -robot_orientation[0] / robot_orientation_sum_squared;
        robot_orientation_inverse_v[1] = -robot_orientation[1] / robot_orientation_sum_squared;
        robot_orientation_inverse_v[2] = -robot_orientation[2] / robot_orientation_sum_squared;

        Vector3d direction_robot_frame = direction_vector + 2*robot_orientation_inverse_w*cross(robot_orientation_inverse_v, direction_vector) +
        2*cross(robot_orientation_inverse_v, cross(robot_orientation_inverse_v, direction_vector));

        cout << "direction_robot_frame: " << direction_robot_frame << endl;*/

        reachable = isReachable(direction_vector);//_robot_frame);
        cout << "Reachable: " << reachable << endl;

        if(!reachable){
            cout<< "goal point not reachable" << endl;
            //find which sub goal is closest to goal.
            // if it is reachable choose that sub goal; otherwise check the next one.
            //hmm what is the best way to do this...

            //Start with transforming the sub goals to cartesian coordinates...
            //or wait, we can also just transform the goal point to spherical coordinates..
            //or just neither dumbass..

            Vector3d subgoal_global_frame, subgoal_robot_frame, robot_orientation_v;
            float robot_orientation_w;
            /*robot_orientation_v [0] = robot_orientation[0];
            robot_orientation_v [1] = robot_orientation[1];
            robot_orientation_v [2] = robot_orientation[2];
            robot_orientation_w = robot_orientation[3];*/
            robot_orientation_v [0] = robot_sphere_matrix_orientation[0];
            robot_orientation_v [1] = robot_sphere_matrix_orientation[1];
            robot_orientation_v [2] = robot_sphere_matrix_orientation[2];
            robot_orientation_w = robot_sphere_matrix_orientation[3];



          	//cout << "done sorting sub goals by distance to goal, 715. " ;   // anyways
            countx = 0;
            for (m = 0; m < spherical_matrix_height; m++){
                for (n = 0; n < spherical_matrix_width; n++){
                    if(subgoal_matrix[m][n] > 0){
                        vector <double> row;
                        /*theta = ((m - 29) *3 - 1.5)*pi/180;
                        phi = ((n-59)*3 - 1.5)*pi/180;
                        x1 = subgoal_matrix[m][n]*sin(phi)*cos(theta);
                        y1= subgoal_matrix[m][n]*sin(phi)*sin(theta);
                        z1= subgoal_matrix[m][n]*cos(phi);*/


                        //NEED to account for orientation of robot w.r.t. global frame..... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                        // Also, currently we are putting direction of goal in global frame into isreachable function
                        // but the direction of sub goals in the robot frame into that function.... !!! this we need to fix
                        rho = subgoal_matrix[m][n];
                        theta = n *  pi / M - pi;
                        phi = m * pi / M - pi/2;

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

                        
                        //z2 = subgoal_global_frame[2]+robot_position[2];
						z2 = subgoal_global_frame[2]+robot_sphere_matrix_position[2];
                                                

                        if(z2 > 0.1){
                        	//this means it is at least 10 cm above ground..

                        	//x2 = subgoal_global_frame[0]+robot_position[0];
                        	//y2 = subgoal_global_frame[1]+robot_position[1];
                        	x2 = subgoal_global_frame[0]+robot_sphere_matrix_position[0];
                        	y2 = subgoal_global_frame[1]+robot_sphere_matrix_position[1];
                        	
                        	// store matrix value
                        	double distance_sq = pow(goal_position[0] - x2,2) +
                        	pow(goal_position[1] - y2, 2) +
                        	pow(goal_position[2] - z2, 2);


							// Change this if you don't want to account for rotation: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        	// try adding one 1cm for each n maybe to account for rotation as well...
                        	double distance = sqrt ( distance_sq) + 0.01* abs(n - N/2);

                        	//now push back to vector..
	                        distance_index_vector.push_back(make_pair(distance, countx));
	                        // push back the m and n values as well..
	                        //matrix_indices_vector.push_back(make_pair(m,n));

	                        row.push_back((double)m);
	                        row.push_back((double)n);
	                        //push bacck robot frame values
	                        row.push_back(x1);
	                        row.push_back(y1);
	                        row.push_back(z1);
	                        // push back global orientation with robot as zero point..
	                        row.push_back(subgoal_global_frame[0]);
	                        row.push_back(subgoal_global_frame[1]);
	                        row.push_back(subgoal_global_frame[2]);
	                        // also push back global values .. 
	                        row.push_back(x2);
	                        row.push_back(y2);
	                        row.push_back(z2);

	                        subgoal_xyz.push_back(row);
	                        countx++;
                        }

                        
                    }
                }
            }

            //cout << "here" << endl;

            // sort stuff
            // there should also be a penalty for choosing a sub goal that is only a little closer to the goal but we have to turn the robot a lot...
            // since without such a penalty we can end up switching between sub goals with very different directions but around the same distance to the goal
            // which makes us turn back and forth but never fly straight... 
            sort(distance_index_vector.begin(), distance_index_vector.end(), comparator());



            // print shit to see if works:

            //for(distance_and_index p : distance_index_vector){
            /*
            for(vitr = distance_index_vector.begin(); vitr != distance_index_vector.end(); ++vitr ){
                    cout << vitr->first << " " << vitr->second << " | ";
            }
            cout << endl;
*/
            for(vector<int>::size_type vecitr = 0; vecitr != distance_index_vector.size(); vecitr++){
                // make vector to
                subgoal_vector[0] = subgoal_xyz[distance_index_vector[vecitr].second][5];
                subgoal_vector[1] = subgoal_xyz[distance_index_vector[vecitr].second][6];
                subgoal_vector[2] = subgoal_xyz[distance_index_vector[vecitr].second][7];
                //if subgoal reachable:

                //if(vecitr == distance_index_vector.size()-1) cout  << "Last sub goal " ;
                //cout << "cheking subgoal " << vecitr << endl;
                if(isReachable(subgoal_vector)){
                    // This subgoal is reachable and therefore the selected subgoal

                    //cout <<"subgoal at squared and normal distance: " << distance_index_vector[vecitr].first<< "  "
                    //<< sqrt(distance_index_vector[vecitr].first) << " chosen" << endl;
                    //DO STUFF HERE!!!!

                    // maybe turn into point cloud for debug reasons...

                    PointCloud::Ptr msg (new PointCloud);
                    msg->header.frame_id = "base_link";
                    msg->height = 1;
                    msg->width = 1;

                    x1 = subgoal_xyz[distance_index_vector[vecitr].second][2];
                    y1 = subgoal_xyz[distance_index_vector[vecitr].second][3];
                    z1 = subgoal_xyz[distance_index_vector[vecitr].second][4];
                    msg->points.push_back (pcl::PointXYZ(x1,y1,z1));

                    // Convert to ROS data type
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*msg, output);
                    // Publish the data
                    pub_selected_subgoal.publish (output);

                    //cout << "subgoal, global orientation: " << subgoal_vector << endl;
                    cout << "subgoal number " << vecitr << " selected" << endl;
                    // Transform to global frame before publishing to topic.  
                    target_vector.x = subgoal_xyz[distance_index_vector[vecitr].second][5];
		    		target_vector.y = subgoal_xyz[distance_index_vector[vecitr].second][6];
		    		target_vector.z = subgoal_xyz[distance_index_vector[vecitr].second][7];

		    		//pub_target_vector.publish(target_vector);

		    		subgoal_point = true;


                    break;
                }
                if(vecitr == distance_index_vector.size() - 1) nothing_reachable = true;

            }
        	cout << endl;
    	}
    	else{
    		// now the main goal is reachable..
    		target_vector.x = direction_vector[0];
    		target_vector.y = direction_vector[1];
    		target_vector.z = direction_vector[2];

    		//pub_target_vector.publish(target_vector);

    		subgoal_point = false;
    	}
    	target_calculated = true;
	    /*}
	    else{
	    	reachable = false;
	    	target_vector.x = direction_vector[0];
	    	target_vector.y = direction_vector[1];
	    	target_vector.z = direction_vector[2];
	    	pub_target_vector.publish(target_vector);

	    	subgoal_point = false;
	    		
	    } */


	    //cout << "yolo" << endl;
	    //now to keep the robot in same place but only turn it:

	    //set_pose.pose.position =  input->pose.position;
	    /*set_pose.pose.position.x = robot_position[0];
	    set_pose.pose.position.y = robot_position[1];
	    set_pose.pose.position.z = robot_position[2];*/


	    //for the purposes of visualization test in rviz, remove later maybe:
	    //if(robot_position[2]<0.4){//input->pose.position.z < 0.3){
	    // to get rid of drift, set fixed position:
    }
    else{
    	// Goal is over or under robot
    	bool reachable;
        
        double theta = acos(direction_vector[0]/xy_length_of_direction_vector);
        if(direction_vector[1] < 0){
                theta = -theta;
        }

        reachable = isReachable(direction_vector);//_robot_frame);
        cout << "Reachable: " << reachable << endl;

        if(!reachable){
            //cout<< "not reachable" << endl;
            //find which sub goal is closest to goal.
            // if it is reachable choose that sub goal; otherwise check the next one.
            //hmm what is the best way to do this...

            //Start with transforming the sub goals to cartesian coordinates...
            //or wait, we can also just transform the goal point to spherical coordinates..
            //or just neither dumbass..

            Vector3d subgoal_global_frame, subgoal_robot_frame, robot_orientation_v;
            float robot_orientation_w;
            robot_orientation_v [0] = robot_sphere_matrix_orientation[0];
            robot_orientation_v [1] = robot_sphere_matrix_orientation[1];
            robot_orientation_v [2] = robot_sphere_matrix_orientation[2];
            robot_orientation_w = robot_sphere_matrix_orientation[3];



          //cout << "done sorting sub goals by distance to goal, 715. " ;   // anyways
            countx = 0;
            for (m = 0; m < spherical_matrix_height; m++){
                for (n = 0; n < spherical_matrix_width; n++){
                    if(subgoal_matrix[m][n] > 0){
                        vector <double> row;
                        /*theta = ((m - 29) *3 - 1.5)*pi/180;
                        phi = ((n-59)*3 - 1.5)*pi/180;
                        x1 = subgoal_matrix[m][n]*sin(phi)*cos(theta);
                        y1= subgoal_matrix[m][n]*sin(phi)*sin(theta);
                        z1= subgoal_matrix[m][n]*cos(phi);*/


                        //NEED to account for orientation of robot w.r.t. global frame..... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                        // Also, currently we are putting direction of goal in global frame into isreachable function
                        // but the direction of sub goals in the robot frame into that function.... !!! this we need to fix
                        rho = subgoal_matrix[m][n];
                        theta = n *  pi / M - pi;
                        phi = m * pi / M - pi/2;

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

                        
                        z2 = subgoal_global_frame[2]+robot_sphere_matrix_position[2];
                        

                        if(z2 > 0.1){
                        	//this means it is at least 10 cm above ground..

                        	x2 = subgoal_global_frame[0]+robot_sphere_matrix_position[0];
                        	y2 = subgoal_global_frame[1]+robot_sphere_matrix_position[1];
                        	
                        	// store matrix value
                        	double distance_sq = pow(goal_position[0] - x2,2) +
                        	pow(goal_position[1] - y2, 2) +
                        	pow(goal_position[2] - z2, 2);
							
							// Change this if you don't want to account for rotation: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							// try adding one 1cm for each n maybe to account for rotation as well...
                        	double distance = sqrt ( distance_sq) + 0.01* abs(n - N/2);

                        	//now push back to vector..
	                        distance_index_vector.push_back(make_pair(distance, countx));
	                        

	                        // push back the m and n values as well..
	                        //matrix_indices_vector.push_back(make_pair(m,n));

	                        row.push_back((double)m);
	                        row.push_back((double)n);
	                        //push bacck robot frame values
	                        row.push_back(x1);
	                        row.push_back(y1);
	                        row.push_back(z1);
	                        // push back global orientation with robot as zero point..
	                        row.push_back(subgoal_global_frame[0]);
	                        row.push_back(subgoal_global_frame[1]);
	                        row.push_back(subgoal_global_frame[2]);
	                        // also push back global values .. 
	                        row.push_back(x2);
	                        row.push_back(y2);
	                        row.push_back(z2);

	                        subgoal_xyz.push_back(row);
	                        countx++;
                        }

                        
                    }
                }
            }

            //cout << "here" << endl;

            // sort stuff
            // maybe account for orientation as well
            sort(distance_index_vector.begin(), distance_index_vector.end(), comparator());



            // print shit to see if works:

            //for(distance_and_index p : distance_index_vector){
            /*
            for(vitr = distance_index_vector.begin(); vitr != distance_index_vector.end(); ++vitr ){
                    cout << vitr->first << " " << vitr->second << " | ";
            }
            cout << endl;
*/
            for(vector<int>::size_type vecitr = 0; vecitr != distance_index_vector.size(); vecitr++){
                // make vector to
                subgoal_vector[0] = subgoal_xyz[distance_index_vector[vecitr].second][5];
                subgoal_vector[1] = subgoal_xyz[distance_index_vector[vecitr].second][6];
                subgoal_vector[2] = subgoal_xyz[distance_index_vector[vecitr].second][7];
                //if subgoal reachable:

                //if(vecitr == distance_index_vector.size()-1) cout  << "Last sub goal " ;
                //cout << "cheking subgoal " << vecitr << endl;
                if(isReachable(subgoal_vector)){
                    // This subgoal is reachable and therefore the selected subgoal

                    //cout <<"subgoal at squared and normal distance: " << distance_index_vector[vecitr].first<< "  "
                    //<< sqrt(distance_index_vector[vecitr].first) << " chosen" << endl;
                    //DO STUFF HERE!!!!

                    // maybe turn into point cloud for debug reasons...

                    PointCloud::Ptr msg (new PointCloud);
                    msg->header.frame_id = "base_link";
                    msg->height = 1;
                    msg->width = 1;

                    x1 = subgoal_xyz[distance_index_vector[vecitr].second][2];
                    y1 = subgoal_xyz[distance_index_vector[vecitr].second][3];
                    z1 = subgoal_xyz[distance_index_vector[vecitr].second][4];
                    msg->points.push_back (pcl::PointXYZ(x1,y1,z1));

                    // Convert to ROS data type
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*msg, output);
                    // Publish the data
                    pub_selected_subgoal.publish (output);

                    //cout << "subgoal, global orientation: " << subgoal_vector << endl;
                    cout << "subgoal number " << vecitr << " selected" << endl;
                    // Transform to global frame before publishing to topic.  
                    target_vector.x = subgoal_xyz[distance_index_vector[vecitr].second][5];
		    		target_vector.y = subgoal_xyz[distance_index_vector[vecitr].second][6];
		    		target_vector.z = subgoal_xyz[distance_index_vector[vecitr].second][7];

		    		//pub_target_vector.publish(target_vector);

		    		subgoal_point = true;


                    break;
                }
                if(vecitr == distance_index_vector.size() - 1) nothing_reachable = true;

            }
        	cout << endl;
    	}
    	else{
    		// now the main goal is reachable..
    		target_vector.x = direction_vector[0];
    		target_vector.y = direction_vector[1];
    		target_vector.z = direction_vector[2];

    		//pub_target_vector.publish(target_vector);

    		subgoal_point = false;
    	}
    	target_calculated = true;

    }  
	//cout << "direction_vector: " << direction_vector << endl;

    //cout <<  "subgoal_point: " << subgoal_point << endl;
    //cout << "target calculated: " << target_calculated << endl;

	//------------------------------------------------------MOTION COMPUTATION--------------------------------------------------

    // First we need to devide the space (spherical matrix) into subspaces..

    // First of all devide it into top / down - left / right..

    // Normals to planes A, B and C are
    Vector3d  target_direction, u_targ, u_sol;
    int m1,n1;

    if(target_calculated){

	    if(!nothing_reachable){
	    	if(subgoal_point){
		    	target_direction[0] = x1;
		    	target_direction[1] = y1;
		    	target_direction[2] = z1;
		    }
		    else{
		    	// find direction vector to goal point in robot frame
		    	Vector3d robot_orientation_inverse_v;
		        float robot_orientation_inverse_w;

		        /*float robot_orientation_sum_squared = pow(robot_orientation[0],2) + pow(robot_orientation[1],2) + pow(robot_orientation[2],2)
		        +pow(robot_orientation[3],2);

		        robot_orientation_inverse_w = robot_orientation[3] / robot_orientation_sum_squared;

		        robot_orientation_inverse_v[0] = -robot_orientation[0] / robot_orientation_sum_squared;
		        robot_orientation_inverse_v[1] = -robot_orientation[1] / robot_orientation_sum_squared;
		        robot_orientation_inverse_v[2] = -robot_orientation[2] / robot_orientation_sum_squared;*/

		        float robot_orientation_sum_squared = pow(robot_sphere_matrix_orientation[0],2) + pow(robot_sphere_matrix_orientation[1],2) + pow(robot_sphere_matrix_orientation[2],2)
		        +pow(robot_sphere_matrix_orientation[3],2);

		        robot_orientation_inverse_w = robot_sphere_matrix_orientation[3] / robot_orientation_sum_squared;

		        robot_orientation_inverse_v[0] = -robot_sphere_matrix_orientation[0] / robot_orientation_sum_squared;
		        robot_orientation_inverse_v[1] = -robot_sphere_matrix_orientation[1] / robot_orientation_sum_squared;
		        robot_orientation_inverse_v[2] = -robot_sphere_matrix_orientation[2] / robot_orientation_sum_squared;

		        target_direction = direction_vector + 2*robot_orientation_inverse_w*cross(robot_orientation_inverse_v, direction_vector) +
		        2*cross(robot_orientation_inverse_v, cross(robot_orientation_inverse_v, direction_vector));



		        //cout << "u_targ: " << u_targ << endl;
		    }

		    float target_distance = vectorlength(target_direction);
		    u_targ[0] = target_direction[0] / target_distance;
			u_targ[1] = target_direction[1] / target_distance;
			u_targ[2] = target_direction[2] / target_distance;

			/*u_targ_vector.x = u_targ[0]; 
			u_targ_vector.y = u_targ[1]; 
			u_targ_vector.z = u_targ[2]; 
			pub_u_targ.publish(u_targ_vector);*/

			Vector3d n_A(0, 1, 0),  //e_y in robot frame 
			e_z(0,0,1), n_B, n_C, n_D;
			n_B = cross(e_z, u_targ);
			n_C = cross(u_targ, n_A);
			Vector3d u, u_obst;

			//  Matrix of directions as quadrants:
			// lets say TR 0, DR 1, DL 2, TL 3;
			int quadrant_matrix [M][N] = {0};
			bool S_nD_TR[M][N] = {false}, S_nD_TL[M][N] = {false}, S_nD_DL[M][N] = {false}, S_nD_DR[M][N] = {false}, S_nD[M][N] = {false};
			bool S_TR_bound[M][N] = {false}, S_TL_bound[M][N] = {false}, S_DL_bound[M][N] = {false}, S_DR_bound[M][N] = {false};
			bool conda, condb, condc;
			

			//cout << "n_C: " << n_C << endl;
			// breyta thessu:
			float theta_u_targ = atan2(u_targ[1] , u_targ[0]);  // now in range  -pi to pi.. do something more.. 
			//if(theta_u_targ < 0) theta_u_targ += pi;
			// I think it's done this way : 
			//if(u_targ[1] > 0){

			// -----------------------Iterate over sphere matrix to divide it into quarters-----------------------
			for(m = 0; m<M; m++){
				for(n=0;n<N;n++){
					// check if obstacle located in direction:
					//if(sphere_matrix[m][n] > 0){
					conda = (n >= N/2);
					
					theta = n *  pi / M - pi;

					// og thessu
					if(theta_u_targ > 0){
						condb = (theta > theta_u_targ || theta < theta_u_targ - pi);	
					}
					else{
						condb = (theta > theta_u_targ && theta < theta_u_targ + pi);
					}

	                u[0] = unitary_direction_vectors_matrix[m][n][0];
	                u[1] = unitary_direction_vectors_matrix[m][n][1];
	                u[2] = unitary_direction_vectors_matrix[m][n][2];

	                condc = dot(u,n_C) > 0;

	                //n_D = cross ( cross(u_targ, u_obst), u_obst); 
					
					if(u_targ[1] > 0){
						// same configuration as in paper 
						if(conda && condb){
							if(condc){
								quadrant_matrix[m][n] = 3; // TL
							}
							else quadrant_matrix[m][n] = 2; // DL
						}
						else
						{
							if(!condc) quadrant_matrix[m][n] = 1; // DR
						}
					}
					else{
						// different configuration than paper
						if(!conda && !condb){
							if(!condc) quadrant_matrix[m][n] = 1; // DR
						}
						else{
							if(condc) quadrant_matrix[m][n] = 3; //TL
							else quadrant_matrix[m][n] = 2; //DL
						}
					}
					//}	
				}
			}



			//}

			// write out quadrant matrix
			/*cout << "quadrant_matrix: " << endl;
			for(m = 0; m<M; m++){
				for(n=0;n<N;n++){
					cout << quadrant_matrix[m][n]<< " ";
				}
				cout << endl;
			}*/

			// -----------------------------Now find S1 and S2: ---------------------------------
			//Vector3d n_D;
			float gamma, gamma_i, obst_dist;//, alpha;
			int m_gamma, m_min, m_max;
			float matrix_half_radian_resolution = spherical_matrix_degree_resolution * pi / 360;
			vector< vector<double> > n_D_vectors_TR,n_D_vectors_TL, n_D_vectors_DR, n_D_vectors_DL;
			vector< vector<double> > bad_directions_TR,bad_directions_TL, bad_directions_DR, bad_directions_DL;

			
			// Iterate over spherical matrix to find S2 and n_D vectors for each quarter:
			int m_temp_min=1;
			/*if(range_down < 0.1){
				m_temp_min = 0; // to get rid of shit values at m = 0
				cout << "range_down: " << range_down << endl;	
			} 
			else m_temp_min= 1;*/



			bool bad_angle = false;
			for(m = m_temp_min; m<M; m++){
				for(n=0;n<N;n++){
					// check if obstacle is present in the direction, 
					// and at a shorter distance than the distance to the goal + robot diameter:
					// this is to make sure the robot doesn't count a wall behind the target as an obstacle
					if (sphere_matrix[m][n] != 0  && sphere_matrix[m][n] < target_distance + robot_radius){ // maybe should use robot_diameter instead of robot_radius here
						//find u_obst, unitary vector in obstacle direction ...
						phi = m * pi / M - pi/2;
						theta = n *  pi / M - pi;
		                r =cos(phi);
		                u_obst[0] = r * cos(theta);
		                u_obst[1] = r * sin(theta);
		                u_obst[2] = sin(phi);
		                obst_dist = vectorlength(u_obst);
		                vector<double> row1;
						n_D = cross ( cross(u_targ, u_obst), u_obst);
						row1.push_back(n_D[0]);
						row1.push_back(n_D[1]);
						row1.push_back(n_D[2]);
						
						gamma = abs(atan((robot_radius + safety_distance)/sphere_matrix[m][n]));
						
						if(sphere_matrix[m][n] < safety_distance + robot_radius){
							gamma += (pi - gamma)*(1 - (sphere_matrix[m][n]-robot_radius)/safety_distance);
							// add angle to some collective bad direction for each quarter.......
							bad_angle = true;
							//cout << "bad angle at m: " << m << "and n: " << n << endl;
							//cout << "distance at bad angle: " << sphere_matrix[m][n];
							}
						 
						//check what quadrant obstacle is in.. TR 0, DR 1, DL 2, TL 3;
						if(quadrant_matrix[m][n] > 1){
							// in left quadrant (TL or DL)

							if(quadrant_matrix[m][n] == 2){
								// in DL
								n_D_vectors_DL.push_back(row1);
								//hmm think about the max angle... 
								// gamma is the angle so 
								m_gamma = ceil(gamma * M / pi);
								m_min = m - m_gamma;
								m_max = m + m_gamma;
								if(m_min < 0 || m_max > M ){
									m_max = M;
									m_min = 0;
								}
								//cout << " N: " << N << " ";
								//cout << "m_min: " << m_min << " m_max: " << m_max << endl;
								//for(int i =0; i < M*N; i++){}
								for(m1=m_min;m1<m_max;m1++){ // this is possibly not the fastest way.....
									for(n1=0;n1<N;n1++){
										u[0] = unitary_direction_vectors_matrix[m1][n1][0];
										u[1] = unitary_direction_vectors_matrix[m1][n1][1];
										u[2] = unitary_direction_vectors_matrix[m1][n1][2];
										gamma_i = abs(acos(dot(u,u_obst)));
										if(abs(gamma_i - gamma) < matrix_half_radian_resolution){
											//this means we're on the boundary of S_2
											S_DL_bound[m1][n1] = true;
										}
										else if(gamma_i < gamma){
											// this means we are within S_2
											S_nD_DL[m1][n1]=true;
										}
									}
								}
								if(bad_angle){
									bad_directions_DL.push_back(row1);
									bad_angle = false;
								}
							}
							else{
								// in TL
								n_D_vectors_TL.push_back(row1);
								m_gamma = ceil(gamma * M / pi);
								m_min = m - m_gamma;
								m_max = m + m_gamma;
								if(m_min < 0 || m_max > M ){
									m_max = M;
									m_min = 0;
								}
								for(m1=m_min;m1<m_max;m1++){
									for(n1=0;n1<N;n1++){
										u[0] = unitary_direction_vectors_matrix[m1][n1][0];
										u[1] = unitary_direction_vectors_matrix[m1][n1][1];
										u[2] = unitary_direction_vectors_matrix[m1][n1][2];
										gamma_i = abs(acos(dot(u,u_obst)));
										if(abs(gamma_i - gamma) < spherical_matrix_degree_resolution * pi / 360){
											//this means we're on the boundary of S_2
											S_TL_bound[m1][n1] = true;
										}
										else if(gamma_i < gamma){
											// this means we are within S_2
											S_nD_TL[m1][n1]=true;
										}
									}
								}
								if(bad_angle){
									bad_directions_TL.push_back(row1);
									bad_angle = false;
								}
							}
						}
						else{
							// in right quadrant

							if(quadrant_matrix[m][n] == 0){
								// in TR
								n_D_vectors_TR.push_back(row1);
								m_gamma = ceil(gamma * M / pi);
								m_min = m - m_gamma;
								m_max = m + m_gamma;
								if(m_min < 0 || m_max > M ){
									m_max = M;
									m_min = 0;
								}
								for(m1=m_min;m1<m_max;m1++){
									for(n1=0;n1<N;n1++){
										u[0] = unitary_direction_vectors_matrix[m1][n1][0];
										u[1] = unitary_direction_vectors_matrix[m1][n1][1];
										u[2] = unitary_direction_vectors_matrix[m1][n1][2];
										gamma_i = abs(acos(dot(u,u_obst)));
										if(abs(gamma_i - gamma) < spherical_matrix_degree_resolution * pi / 360){
											//this means we're on the boundary of S_2
											S_TR_bound[m1][n1] = true;
										}
										else if(gamma_i < gamma){
											// this means we are within S_2
											S_nD_TR[m1][n1]=true;
										}
									}
								}
								if(bad_angle){
									bad_directions_TR.push_back(row1);
									bad_angle = false;
								}

							}
							else{
								// in DR
								n_D_vectors_DR.push_back(row1);
								m_gamma = ceil(gamma * M / pi);
								m_min = m - m_gamma;
								m_max = m + m_gamma;
								if(m_min < 0 || m_max > M ){
									m_max = M;
									m_min = 0;
								}
								for(m1=m_min;m1<m_max;m1++){ 
									for(n1=0;n1<N;n1++){
										u[0] = unitary_direction_vectors_matrix[m1][n1][0];
										u[1] = unitary_direction_vectors_matrix[m1][n1][1];
										u[2] = unitary_direction_vectors_matrix[m1][n1][2];
										gamma_i = abs(acos(dot(u,u_obst)));
										if(abs(gamma_i - gamma) < spherical_matrix_degree_resolution * pi / 360){
											//this means we're on the boundary of S_2
											S_DR_bound[m1][n1] = true;
										}
										else if(gamma_i < gamma){
											// this means we are within S_2
											S_nD_DR[m1][n1]=true;
										}
									}
								}
								if(bad_angle){
									bad_directions_DR.push_back(row1);
									bad_angle = false;
								}
							}
						}
					}
				}
			}

			/*cout << "S_nD_DR without S1: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_nD_DR[m][n] << " ";
				}
				cout<< endl;
			}
*/
			/*cout << "S_nD_DL without S1: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_nD_DL[m][n] << " ";
				}
				cout<< endl;
			}*/
	//}

			/*cout << "S_DL bound before deleting certain points: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_DL_bound[m][n] << " ";
				}
				cout<< endl;
			}*/
			// ------------------------------------Now construct S_1-----------------------------------------

			// We have the n_D vectors for each quadrant
			// construct S1 for each quarter :
			// what we iterate over depends on limits of left and right quarters... 
			Vector3d temp_vec_3d;
			int n_target = (theta_u_targ + pi) * N / 2 / pi;

			// Not sure if following is necessary:
			if(n_target < 0) n_target = 0;
			if(n_target > N) n_target = N; 
			// for TL first : 
			// iterate over left quarters:
			if(n_D_vectors_TL.size() > 0){
				for(m=0;m<M;m++){
					for (n = n_target ; n < N; n++){
						// check if points belongs to D+ of any of the n_D vectors of the TL thing
						for (int i = 0 ; i < n_D_vectors_TL.size(); i++){
							u[0] = unitary_direction_vectors_matrix[m][n][0];
							u[1] = unitary_direction_vectors_matrix[m][n][1];
							u[2] = unitary_direction_vectors_matrix[m][n][2];
							temp_vec_3d[0] = n_D_vectors_TL[i][0];
							temp_vec_3d[1] = n_D_vectors_TL[i][1];
							temp_vec_3d[2] = n_D_vectors_TL[i][2];
							if (dot(temp_vec_3d, u) > 0){
								S_nD_TL[m][n] = true;
								break;	
							} 
						}
					}
				}
			}
			// for DL : 
			// iterate over left quarters:
			if(n_D_vectors_DL.size() > 0){
				for(m=0;m<M;m++){
					for (n = n_target ; n < N; n++){
						// check if points belongs to D+ of any of the n_D vectors
						for (int i = 0 ; i < n_D_vectors_DL.size(); i++){
							u[0] = unitary_direction_vectors_matrix[m][n][0];
							u[1] = unitary_direction_vectors_matrix[m][n][1];
							u[2] = unitary_direction_vectors_matrix[m][n][2];
							temp_vec_3d[0] = n_D_vectors_DL[i][0];
							temp_vec_3d[1] = n_D_vectors_DL[i][1];
							temp_vec_3d[2] = n_D_vectors_DL[i][2];
							if (dot(temp_vec_3d, u) > 0){
								S_nD_DL[m][n] = true;
								break;
							} 
						}
					}
				}
			}
			// for DR : 
			// iterate over right quarters:
			if(n_D_vectors_DR.size() > 0){
				for(m=0;m<M;m++){
					for (n = 0 ; n < n_target; n++){
						// check if points belongs to D+ of any of the n_D vectors 
						for (int i = 0 ; i < n_D_vectors_DR.size(); i++){
							u[0] = unitary_direction_vectors_matrix[m][n][0];
							u[1] = unitary_direction_vectors_matrix[m][n][1];
							u[2] = unitary_direction_vectors_matrix[m][n][2];
							temp_vec_3d[0] = n_D_vectors_DR[i][0];
							temp_vec_3d[1] = n_D_vectors_DR[i][1];
							temp_vec_3d[2] = n_D_vectors_DR[i][2];
							if (dot(temp_vec_3d, u) > 0){
								S_nD_DR[m][n] = true;
								break;	
							} 
						}
					}
				}
			}
			// for TR : 
			// iterate over right quarters:
			if(n_D_vectors_TR.size() > 0){
				for(m=0;m<M;m++){
					for (n = 0 ; n < n_target; n++){
						// check if points belongs to D+ of any of the n_D vectors 
						for (int i = 0 ; i < n_D_vectors_TR.size(); i++){
							u[0] = unitary_direction_vectors_matrix[m][n][0];
							u[1] = unitary_direction_vectors_matrix[m][n][1];
							u[2] = unitary_direction_vectors_matrix[m][n][2];
							temp_vec_3d[0] = n_D_vectors_TR[i][0];
							temp_vec_3d[1] = n_D_vectors_TR[i][1];
							temp_vec_3d[2] = n_D_vectors_TR[i][2];
							if (dot(temp_vec_3d, u) > 0){
								S_nD_TR[m][n] = true;
								break;	
							} 
						}
					}
				}
			}

			// ---------- Iterate over matrix to remove boundary that should not be counted as such -----------

			// perhaps also remove boundary that is out of sight for the sensors .... ? 

			vector< vector<double> > S_TR_bound_vectors,S_TL_bound_vectors, S_DR_bound_vectors, S_DL_bound_vectors;


		  	//Remove boundary points that are not in field of view of camera.. 
		  	int m_camera_min = M * 28/60;
		  	int m_camera_max = M * 40/60;

			for(m = 0; m < M ; m ++){
				for (n = 0; n < N; n++){
					//countx = 0;
					// For TL:
					if(S_TL_bound[m][n]){
						// check if free or not.. 
						if(S_nD_TL[m][n] || m < m_camera_min && m > 1 || m > m_camera_max && m < M-2){
							S_TL_bound[m][n] = false; // if not free, should not be on boundary.. 
						} 
						else{
							// store where the boundary directions are to use later on...
							vector < double > row2;
							row2.push_back(unitary_direction_vectors_matrix[m][n][0]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][1]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][2]);
							S_TL_bound_vectors.push_back(row2); 

							//cout << "row of S_TL_bound_vectors: " << row2[0] << " " << row2[1] << " " << row2[2] << endl;
							/*cout << "corresponding unitary_direction_vectors_matrix thing : " ;
							cout << unitary_direction_vectors_matrix[m][n][0] << " "<< unitary_direction_vectors_matrix[m][n][1] ;
							cout << " " << unitary_direction_vectors_matrix[m][n][2] << endl;*/
						}
					}
					// For DL:
					if(S_DL_bound[m][n]){
						if(S_nD_DL[m][n] || m < m_camera_min && m > 1 || m > m_camera_max && m < M-2) S_DL_bound[m][n] = false;
						else{
							// store where the boundary directions are to use later on...
							vector < double > row2;
							row2.push_back(unitary_direction_vectors_matrix[m][n][0]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][1]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][2]);
							S_DL_bound_vectors.push_back(row2); 
						}
					}
					// For DR:
					if(S_DR_bound[m][n] ){
						if(S_nD_DR[m][n]|| m < m_camera_min && m > 1 || m > m_camera_max && m < M-2) S_DR_bound[m][n] = false;
						else{
							// store where the boundary directions are to use later on...
							vector < double > row2;
							row2.push_back(unitary_direction_vectors_matrix[m][n][0]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][1]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][2]);
							S_DR_bound_vectors.push_back(row2); 
						}
					}
					// For TR:
					if(S_TR_bound[m][n] ){
						if(S_nD_TR[m][n]|| m < m_camera_min && m > 1 || m > m_camera_max && m < M-2) S_TR_bound[m][n] = false;
						else{
							// store where the boundary directions are to use later on...
							vector < double > row2;
							row2.push_back(unitary_direction_vectors_matrix[m][n][0]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][1]);
							row2.push_back(unitary_direction_vectors_matrix[m][n][2]);
							S_TR_bound_vectors.push_back(row2); 
						}
					}
				}
			}



			// OK, so now I have the S_nD for each quarter....

			// maybe let's print out the S_nD matrix for debug purposes... 
			/*cout << "S_nD_DR: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_nD_DR[m][n] << " ";
				}
				cout<< endl;
			}

			cout << "S_nD_DL: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_nD_DL[m][n] << " ";
				}
				cout<< endl;
			}*/

			/*cout << "S_nD_DL: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_nD_DL[m][n] << " ";
				}
				cout<< endl;
			}*/

			/*cout << "S_DL bound after: " << endl;
			for(m=M-1;m>=0;m--){
				for(n=N-1;n>=0;n--){
				cout << S_DL_bound[m][n] << " ";
				}
				cout<< endl;
			}*/

			// ---------------------- Now find the most promising direction of motion ---------------------------

			// Check if there exist any free directions:
			bool free_direction = false;
			for(m=0;m<M;m++){
				for(n=0;n<N;n++){
					if(!S_nD_DL[m][n] && !S_nD_DR[m][n] && !S_nD_TL[m][n] && !S_nD_TR[m][n]){
						// this direction is free
						free_direction = true;
						break;
					}
				}
			}

			// find loction of target direction in matrix ... 

		    theta = atan2(u_targ[1], u_targ[0]);
		    phi = asin(u_targ[2]);  

		    int m_targ = (phi + pi/2)*M/pi;
		    int n_targ = (theta + pi)*M/pi;

		    int u_targ_S_nD_count =0;

		    /*if(S_nD_TL[m_targ][n_targ]) u_targ_S_nD_count++;
		    if(S_nD_TR[m_targ][n_targ]) u_targ_S_nD_count++;
		    if(S_nD_DL[m_targ][n_targ]) u_targ_S_nD_count++;
		    if(S_nD_DR[m_targ][n_targ]) u_targ_S_nD_count++;*/		    

		    if(m_targ < 0) m_targ = 0;
		    if(n_targ < 0) n_targ = 0;
		    if(m_targ >= M) m_targ = M-1;
		    if(n_targ >= N) n_targ = N-1;	

		    


		    //else{

	    	Vector3d u_TL_dom, u_TR_dom, u_DL_dom, u_DR_dom;
	    	float current_smallest_angle, current_angle;
	    	bool use_u_TL_dom = false,use_u_DR_dom = false,use_u_DL_dom = false,use_u_TR_dom = false;

	    	// write out S_TL_bound_vectors:

	    	/*cout << "S_TL_bound_vectors: " <<endl;
	    	for(int i = 1; i < S_TL_bound_vectors.size(); i++){
	    		cout << S_TL_bound_vectors[i][0] << " "<< S_TL_bound_vectors[i][1] << " " << S_TL_bound_vectors[i][2] << endl;
	    	}*/

		    if(free_direction){
				// ---------------------------------------This means there exist free directions---------------------------------------
				if(S_nD_TL[m_targ][n_targ]){
					// calculate u^TL_dom, the direction on the boundary that is closest to the target direction...

					if(S_TL_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_TL_dom[0] = S_TL_bound_vectors[0][0];
						u_TL_dom[1] = S_TL_bound_vectors[0][1];
						u_TL_dom[2] = S_TL_bound_vectors[0][2];
						/*cout << "initial u_TL_dom: " << u_TL_dom;
						cout << "initial S_TL_bound_vectors: " << S_TL_bound_vectors[0][0] <<" "<< S_TL_bound_vectors[0][1] << " " ;*/
						//cout << S_TL_bound_vectors[0][2] << endl;
						current_smallest_angle = abs(acos(dot(u_TL_dom, u_targ)));
						for(int i = 1; i < S_TL_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_TL_bound_vectors[i][0];
							temp_vec_3d[1] = S_TL_bound_vectors[i][1];
							temp_vec_3d[2] = S_TL_bound_vectors[i][2];

							current_angle = abs(acos(dot(temp_vec_3d, u_targ)));
							if(current_angle < current_smallest_angle){
								u_TL_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_TL_dom = true;
						u_targ_S_nD_count++;
						//cout << "final u_TL_dom: " << u_TL_dom;
					}
					else{
						cout << "bound directions for TL < 2" << endl;
					}
				}
				if(S_nD_TR[m_targ][n_targ]){
					// calculate u^TL_dom, the direction on the boundary that is closest to the target direction...

					if(S_TR_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_TR_dom[0] = S_TR_bound_vectors[0][0];
						u_TR_dom[1] = S_TR_bound_vectors[0][1];
						u_TR_dom[2] = S_TR_bound_vectors[0][2];
						current_smallest_angle = abs(acos(dot(u_TR_dom, u_targ)));
						for(int i = 1; i < S_TR_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_TR_bound_vectors[i][0];
							temp_vec_3d[1] = S_TR_bound_vectors[i][1];
							temp_vec_3d[2] = S_TR_bound_vectors[i][2];

							current_angle = abs(acos(dot(temp_vec_3d, u_targ)));
							if(current_angle < current_smallest_angle){
								u_TR_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_TR_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for TR < 2" << endl;
					}					
				}
				if(S_nD_DL[m_targ][n_targ]){
					// calculate u^DL_dom, the direction on the boundary that is closest to the target direction...

					if(S_DL_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_DL_dom[0] = S_DL_bound_vectors[0][0];
						u_DL_dom[1] = S_DL_bound_vectors[0][1];
						u_DL_dom[2] = S_DL_bound_vectors[0][2];
						current_smallest_angle = abs(acos(dot(u_DL_dom, u_targ)));
						for(int i = 1; i < S_DL_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_DL_bound_vectors[i][0];
							temp_vec_3d[1] = S_DL_bound_vectors[i][1];
							temp_vec_3d[2] = S_DL_bound_vectors[i][2];

							current_angle = abs(acos(dot(temp_vec_3d, u_targ)));
							if(current_angle < current_smallest_angle){
								u_DL_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_DL_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for DL < 2" << endl;
					}					
				}
				if(S_nD_DR[m_targ][n_targ]){
					// calculate u^DR_dom, the direction on the boundary that is closest to the target direction...

					if(S_DR_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_DR_dom[0] = S_DR_bound_vectors[0][0];
						u_DR_dom[1] = S_DR_bound_vectors[0][1];
						u_DR_dom[2] = S_DR_bound_vectors[0][2];
						current_smallest_angle = abs(acos(dot(u_DR_dom, u_targ)));
						for(int i = 1; i < S_DR_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_DR_bound_vectors[i][0];
							temp_vec_3d[1] = S_DR_bound_vectors[i][1];
							temp_vec_3d[2] = S_DR_bound_vectors[i][2];

							current_angle = abs(acos(dot(temp_vec_3d, u_targ)));
							if(current_angle < current_smallest_angle){
								u_DR_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_DR_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for DR < 2" << endl;
					}					
				}
			}
			else{
				// ------------------------------------------This means there is no free direction-----------------------------------
				//Vector3d 
				vector <double> sum_vector;
				if(S_nD_TL[m_targ][n_targ]){
					// calculate u^TL_dom, the direction on the boundary that is closest to the target direction...

					if(S_TL_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_TL_dom[0] = S_TL_bound_vectors[0][0];
						u_TL_dom[1] = S_TL_bound_vectors[0][1];
						u_TL_dom[2] = S_TL_bound_vectors[0][2];
						current_smallest_angle = abs(acos(u_TL_dom[0])); // since this is result of dot product with (1,0,0)
						for(int i = 1; i < S_TL_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_TL_bound_vectors[i][0];
							temp_vec_3d[1] = S_TL_bound_vectors[i][1];
							temp_vec_3d[2] = S_TL_bound_vectors[i][2];

							current_angle = abs(acos(temp_vec_3d[0])); // since this is result of dot product with (1,0,0)
							if(current_angle < current_smallest_angle){
								u_TL_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_TL_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for TL < 2, no free directions" << endl;
						// in this case calculate u_TL_dom from the bad directions... 
						sumx=0, sumy=0, sumz=0;
						if(bad_directions_TL.size() > 0){
							for (int i = 0 ; i < bad_directions_TL.size(); i++){
								//sum_vector += bad_directions_TL[i];
								sumx +=bad_directions_TL[i][0];
								sumy +=bad_directions_TL[i][1];
								sumz +=bad_directions_TL[i][2];
							}
							//sum_vector /= bad_directions_TL.size();
							sumx /=bad_directions_TL.size();
							sumy /=bad_directions_TL.size();
							sumz /=bad_directions_TL.size();
							
							u_TL_dom[0] = -sumx;
							u_TL_dom[1] = -sumy;
							u_TL_dom[2] = -sumz;
							use_u_TL_dom = true;
							u_targ_S_nD_count++;

							//cout  << "u_TL_dom: " << u_TL_dom << endl;
						}
					}					
				}
				if(S_nD_TR[m_targ][n_targ]){
					// calculate u^TR_dom, the direction on the boundary that is closest to the target direction...

					if(S_TR_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_TR_dom[0] = S_TR_bound_vectors[0][0];
						u_TR_dom[1] = S_TR_bound_vectors[0][1];
						u_TR_dom[2] = S_TR_bound_vectors[0][2];
						current_smallest_angle = abs(acos(u_TR_dom[0])); // since this is result of dot product with (1,0,0)
						for(int i = 1; i < S_TR_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_TR_bound_vectors[i][0];
							temp_vec_3d[1] = S_TR_bound_vectors[i][1];
							temp_vec_3d[2] = S_TR_bound_vectors[i][2];

							current_angle = abs(acos(temp_vec_3d[0])); // since this is result of dot product with (1,0,0)
							if(current_angle < current_smallest_angle){
								u_TR_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_TR_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for TR < 2, no free directions" << endl;
						// in this case calculate u_TR_dom from the bad directions... 
						sumx=0, sumy=0, sumz=0;
						if(bad_directions_TR.size() > 0){
							for (int i = 0 ; i < bad_directions_TR.size(); i++){
								//sum_vector += bad_directions_TL[i];
								sumx +=bad_directions_TR[i][0];
								sumy +=bad_directions_TR[i][1];
								sumz +=bad_directions_TR[i][2];
							}
							//sum_vector /= bad_directions_TL.size();
							sumx /=bad_directions_TR.size();
							sumy /=bad_directions_TR.size();
							sumz /=bad_directions_TR.size();
							
							u_TR_dom[0] = -sumx;
							u_TR_dom[1] = -sumy;
							u_TR_dom[2] = -sumz;
							use_u_TR_dom = true;
							u_targ_S_nD_count++;
						}
					}					
				}
				if(S_nD_DR[m_targ][n_targ]){
					// calculate u^DR_dom, the direction on the boundary that is closest to the target direction...

					if(S_DR_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_DR_dom[0] = S_DR_bound_vectors[0][0];
						u_DR_dom[1] = S_DR_bound_vectors[0][1];
						u_DR_dom[2] = S_DR_bound_vectors[0][2];
						current_smallest_angle = abs(acos(u_DR_dom[0])); // since this is result of dot product with (1,0,0)
						for(int i = 1; i < S_DR_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_DR_bound_vectors[i][0];
							temp_vec_3d[1] = S_DR_bound_vectors[i][1];
							temp_vec_3d[2] = S_DR_bound_vectors[i][2];

							current_angle = abs(acos(temp_vec_3d[0])); // since this is result of dot product with (1,0,0)
							if(current_angle < current_smallest_angle){
								u_DR_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_DR_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for DR < 2, no free directions" << endl;
						// in this case calculate u_TL_dom from the bad directions... 
						sumx=0, sumy=0, sumz=0;
						if(bad_directions_DR.size() > 0){
							for (int i = 0 ; i < bad_directions_DR.size(); i++){
								//sum_vector += bad_directions_TL[i];
								sumx +=bad_directions_DR[i][0];
								sumy +=bad_directions_DR[i][1];
								sumz +=bad_directions_DR[i][2];
							}
							//sum_vector /= bad_directions_TL.size();
							sumx /=bad_directions_DR.size();
							sumy /=bad_directions_DR.size();
							sumz /=bad_directions_DR.size();
							
							u_DR_dom[0] = -sumx;
							u_DR_dom[1] = -sumy;
							u_DR_dom[2] = -sumz;
							use_u_DR_dom = true;
							u_targ_S_nD_count++;
							//cout  << "u_DR_dom: " << u_DR_dom << endl;

						}
					}					
				}
				if(S_nD_DL[m_targ][n_targ]){
					// calculate u^DL_dom, the direction on the boundary that is closest to the target direction...

					if(S_DL_bound_vectors.size() > 1){

						// initialize with first boundary vector:
						u_DL_dom[0] = S_DL_bound_vectors[0][0];
						u_DL_dom[1] = S_DL_bound_vectors[0][1];
						u_DL_dom[2] = S_DL_bound_vectors[0][2];
						current_smallest_angle = abs(acos(u_DL_dom[0])); // since this is result of dot product with (1,0,0)
						for(int i = 1; i < S_DL_bound_vectors.size(); i++){
							// check if closest to target direction
							temp_vec_3d[0] = S_DL_bound_vectors[i][0];
							temp_vec_3d[1] = S_DL_bound_vectors[i][1];
							temp_vec_3d[2] = S_DL_bound_vectors[i][2];

							current_angle = abs(acos(temp_vec_3d[0])); // since this is result of dot product with (1,0,0)
							if(current_angle < current_smallest_angle){
								u_DL_dom = temp_vec_3d;
								current_smallest_angle = current_angle;
							}
						}
						use_u_DL_dom = true;
						u_targ_S_nD_count++;
					}
					else{
						cout << "bound directions for DL < 2, no free directions" << endl;
						// in this case calculate u_TL_dom from the bad directions... 
						sumx=0, sumy=0, sumz=0;
						if(bad_directions_DL.size() > 0){
							for (int i = 0 ; i < bad_directions_DL.size(); i++){
								//sum_vector += bad_directions_TL[i];
								sumx +=bad_directions_DL[i][0];
								sumy +=bad_directions_DL[i][1];
								sumz +=bad_directions_DL[i][2];
							}
							//sum_vector /= bad_directions_TL.size();
							sumx /= bad_directions_DL.size();
							sumy /= bad_directions_DL.size();
							sumz /= bad_directions_DL.size();
							
							u_DL_dom[0] = -sumx;
							u_DL_dom[1] = -sumy;
							u_DL_dom[2] = -sumz;
							use_u_DL_dom = true;
							u_targ_S_nD_count++;
							//cout  << "u_DR_dom: " << u_DR_dom << endl;
						}
					}					
				}
			}

			// -------------------------------Now calculate the most desired direction of motion----------------------
			/*cout << "u_targ: " << u_targ << endl;
			cout << " u_TL_dom: " << u_TL_dom << endl << " u_TR_dom: " << u_TR_dom << endl;
			cout <<" u_DL_dom: " << u_DL_dom << endl << " u_DR_dom: " << u_DR_dom << endl;
			cout << "u_targ_S_nD_count: " << u_targ_S_nD_count << endl;*/


			PointCloud::Ptr msg (new PointCloud);
			msg->header.frame_id = "base_link";
			msg->height = 1;

			if(u_targ_S_nD_count == 0){
				// to visulize u_doms in rviz:
				msg->width = 1;
				msg->points.push_back (pcl::PointXYZ(0,0,0));


		    	// this means we should fly straight to target since it is free
		    	cout << "fly straight to target point" << endl;
		    	u_sol = u_targ;
		    }
			else if(u_targ_S_nD_count == 1){
				msg->width = 1;


				// target direction only belongs to one S_nD of four sets of motion constraints...
				/*if(S_nD_TL[m_targ][n_targ]) u_sol = u_TL_dom;
				else if(S_nD_DL[m_targ][n_targ]) u_sol = u_DL_dom;
				else if(S_nD_TR[m_targ][n_targ]) u_sol = u_TR_dom;
				else if(S_nD_DR[m_targ][n_targ]) u_sol = u_DR_dom;*/
				if(use_u_TL_dom) u_sol = u_TL_dom;
				else if(use_u_DL_dom) u_sol = u_DL_dom;
				else if(use_u_TR_dom) u_sol = u_TR_dom;
				else if(use_u_DR_dom) u_sol = u_DR_dom;
				msg->points.push_back(pcl::PointXYZ(u_sol[0],u_sol[1],u_sol[2]));
			}
			else if(u_targ_S_nD_count == 2){
				// target direction belongs to two sets of motion constraints
				// use average value of those two.. 

				msg->width = 2;

				cout << "target direction belongs to 2 sets of motion constraints" << endl;
				countx = 0;

				/*if(S_nD_TL[m_targ][n_targ]){
					temp_vec_3d = u_TL_dom;
					countx++;
				}
				if(S_nD_DL[m_targ][n_targ]){
					if(countx == 0){
						temp_vec_3d = u_DL_dom;
						countx++;
					}
					else{
						u_sol = (temp_vec_3d + u_DL_dom)/2;
					}
				}
				if(S_nD_TR[m_targ][n_targ]){
					if(countx == 0){
						temp_vec_3d = u_TR_dom;
						countx++;
					}
					else{
						u_sol = (temp_vec_3d + u_TR_dom)/2;
					}
				}
				if(S_nD_DR[m_targ][n_targ]){
					u_sol = (temp_vec_3d + u_DR_dom)/2;
				}*/
				if(use_u_TL_dom){
					temp_vec_3d = u_TL_dom;
					countx++;

					msg->points.push_back(pcl::PointXYZ(u_TL_dom[0],u_TL_dom[1],u_TL_dom[2]));
				}
				if(use_u_DL_dom){
					msg->points.push_back(pcl::PointXYZ(u_DL_dom[0],u_DL_dom[1],u_DL_dom[2]));
					if(countx == 0){
						temp_vec_3d = u_DL_dom;
						countx++;
					}
					else{
						u_sol = (temp_vec_3d + u_DL_dom)/2;
					}
				}
				if(use_u_TR_dom){
					msg->points.push_back(pcl::PointXYZ(u_TR_dom[0],u_TR_dom[1],u_TR_dom[2]));
					if(countx == 0){
						temp_vec_3d = u_TR_dom;
						countx++;
					}
					else{
						u_sol = (temp_vec_3d + u_TR_dom)/2;
					}
				}
				if(use_u_DR_dom){
					u_sol = (temp_vec_3d + u_DR_dom)/2;

					msg->points.push_back(pcl::PointXYZ(u_DR_dom[0],u_DR_dom[1],u_DR_dom[2]));
				}
			}
			else if(u_targ_S_nD_count == 3){

				msg->width = 3;

				// target direction belongs to three sets of motion constraints..
				cout << "target direction belongs to 3 sets of motion constraints" << endl;
				//if(S_nD_TL[m_targ][n_targ] && S_nD_DR[m_targ][n_targ]){
				if(use_u_TL_dom && use_u_DR_dom){
					msg->points.push_back(pcl::PointXYZ(u_TL_dom[0],u_TL_dom[1],u_TL_dom[2]));
					msg->points.push_back(pcl::PointXYZ(u_DR_dom[0],u_DR_dom[1],u_DR_dom[2]));

					if(use_u_TR_dom){
						u_sol = ((u_TL_dom + u_DR_dom)/2 + u_TR_dom) / 2;
						msg->points.push_back(pcl::PointXYZ(u_TR_dom[0],u_TR_dom[1],u_TR_dom[2]));
					}
					else{
						u_sol = ((u_TL_dom + u_DR_dom)/2 + u_DL_dom) / 2;
						msg->points.push_back(pcl::PointXYZ(u_DL_dom[0],u_DL_dom[1],u_DL_dom[2]));
					}
				}
				else{
					msg->points.push_back(pcl::PointXYZ(u_TR_dom[0],u_TR_dom[1],u_TR_dom[2]));
					msg->points.push_back(pcl::PointXYZ(u_DL_dom[0],u_DL_dom[1],u_DL_dom[2]));
					if(use_u_TL_dom){
						u_sol = ((u_TR_dom + u_DL_dom)/2 + u_TL_dom) / 2;

						msg->points.push_back(pcl::PointXYZ(u_TL_dom[0],u_TL_dom[1],u_TL_dom[2]));
					} 
					else{
						u_sol = ((u_TR_dom + u_DL_dom)/2 + u_DR_dom) / 2;

						msg->points.push_back(pcl::PointXYZ(u_DR_dom[0],u_DR_dom[1],u_DR_dom[2]));
					} 
				}
			}
			else{
				msg->width = 4;
				msg->points.push_back(pcl::PointXYZ(u_TL_dom[0],u_TL_dom[1],u_TL_dom[2]));
				msg->points.push_back(pcl::PointXYZ(u_DR_dom[0],u_DR_dom[1],u_DR_dom[2]));
				msg->points.push_back(pcl::PointXYZ(u_TR_dom[0],u_TR_dom[1],u_TR_dom[2]));
				msg->points.push_back(pcl::PointXYZ(u_DL_dom[0],u_DL_dom[1],u_DL_dom[2]));

				// target direction belongs to all four sets of motion constraints..
				cout << "target direction belongs to all 4 sets of motion constraints" << endl;
				/*Vector3d u_TR_DL_dom, u_TL_DR_dom, n_E, n_F;
				
				u_TL_DR_dom = (u_DR_dom + u_TL_dom) / 2;
				u_TR_DL_dom = (u_TR_dom + u_DL_dom) / 2;
				n_E = cross(cross(u_TL_dom, u_DR_dom), u_TL_DR_dom);
				n_F = cross(cross(u_TR_dom, u_DL_dom), u_TR_DL_dom);
				u_sol = cross(n_E, n_F);*/

				// Try using medium value instead:
				u_sol = (u_DR_dom + u_TL_dom + u_TR_dom + u_DL_dom) / 4;
			}

			//cout << "u_sol : " << u_sol << endl;
			// To send vector with correct length instead of unitary vector:
    		cout << "u_sol:" << u_sol << endl;

    		u_sol *= target_distance;

    		// To publish the u_dom vectors to cloud:
    		// Convert to ROS data type
		  	sensor_msgs::PointCloud2 output;
		  	pcl::toROSMsg(*msg, output);
		  	// Publish the data
		  	pub_u_dom_cloud.publish (output);

	    } // end of if (!nothing_reachable)
	    else{
	    	// put something into u_sol that makes it evident it has not been published in a normal way
	    	cout << "nothing_reachable !" << endl;
	    	u_sol[0] = 1000;
	    	u_sol[1] = 0;
	    	u_sol[2] = 0;
	    }
	} // end of if(target_calculated)  
	else{
		cout << "target not calculated!" << endl;
    	// put something into u_sol that makes it evident it has not been published in a normal way
    	u_sol[0] = 1000;
    	u_sol[1] = 0;
    	u_sol[2] = 0;
    }

    //publish u sol rather as a global position since robot may have shifted in orientation / location

    /*u_sol_vector.x = u_sol[0];
    u_sol_vector.y = u_sol[1];
    u_sol_vector.z = u_sol[2];

    pub_u_sol.publish(u_sol_vector);*/


    Vector3d robot_orientation_v(robot_sphere_matrix_orientation[0], robot_sphere_matrix_orientation[1], robot_sphere_matrix_orientation[2]);
    float robot_orientation_w = robot_sphere_matrix_orientation[3];

    Vector3d u_sol_global;
    u_sol_global = u_sol + 2*robot_orientation_w*cross(robot_orientation_v, u_sol) +
    2*cross(robot_orientation_v, cross(robot_orientation_v, u_sol));
	Vector3d target_position;
    target_position[0] = robot_position[0] + u_sol_global[0];
    target_position[1] = robot_position[1] + u_sol_global[1];
    target_position[2] = robot_position[2] + u_sol_global[2];   

    target_position_vector.x = target_position[0];
    target_position_vector.y = target_position[1];
    target_position_vector.z = target_position[2];

    pub_target_position.publish(target_position_vector);

    // Transform to point cloud to visualize in rviz:
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "/world";
    msg->height = 1;
    msg->width = 1;

    msg->points.push_back (pcl::PointXYZ(target_position[0], target_position[1], target_position[2]));

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*msg, output);
    // Publish the data
    pub_u_sol_cloud.publish (output);

}

void MotionComputation::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr& input) {

	robot_position[0] = input->pose.position.x;
	robot_position[1] = input->pose.position.y;
	robot_position[2] = input->pose.position.z;

	robot_orientation[0] = input->pose.orientation.x;
	robot_orientation[1] = input->pose.orientation.y;
	robot_orientation[2] = input->pose.orientation.z;
	robot_orientation[3] = input->pose.orientation.w;


}



void MotionComputation::octoMapCallback(const octomap_msgs::OctomapConstPtr& octomap_msg){

    octomap::AbstractOcTree* oldtree = octomap_msgs::binaryMsgToMap(*octomap_msg);
    tree = (octomap::OcTree*)oldtree;

}


//void MotionComputation::isReachable(const geometry_msgs::Vector3 & direction){  //?????????? veit ekki alveg hvernig er best ad gera thetta... 




//-----------------------------------------Function that tests if point lies within cylinder in 3D:-----------------------------------
//taken from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
/*struct Vec3
{
	float x;
	float y;
	float z;
};*/
// use Vector3 instead of Vec3:

float MotionComputation::CylTest_CapsFirst(const Vector3d & dir_vec, float lengthsq, float  radius_sq, float  pdx, float pdy, float pdz)
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



Vector3d MotionComputation::cross(const Vector3d & vec1, const Vector3d & vec2){
	Vector3d cross_product;
	cross_product[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
	cross_product[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
	cross_product[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
	
	return ( cross_product );
}

float MotionComputation::dot(const Vector3d & vec1, const Vector3d & vec2){
	
	return ( vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2] );
}

float MotionComputation::vectorlength(const Vector3d & vec){
	
	return ( sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]) );
}

int main(int argc, char** argv)
{

// ROS set-ups:
    ros::init(argc, argv, "motion_computation"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type MotionComputation");
    MotionComputation motionComputation(&nh);  //instantiate an MotionComputation object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");

    //Wait a second, maybe do all the work here instead... 

    ros::spin();
	return 0;
}


bool MotionComputation::isReachable(const Vector3d & direction){ 
//geometry_msgs::Vector3 direction){

	//check if direction is reachable from current robot location...

	//cout << "in isReachable function" << endl;

	float length_of_direction_vector_sq  = pow(direction[0],2) + pow(direction[1],2) + pow(direction[2],2);
	float length_of_direction_vector = sqrt(length_of_direction_vector_sq);
	float length_of_direction_vector_robot_width_sq = pow(length_of_direction_vector + robot_radius, 2);

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

	//vector <vector <double> > points_within_matr;

	//vector <vector <double> > obstacle_points_direction_frame;
 
	//double sumx, sumy, sumz = 0.0;
	//int count = 0 ;

	// ------------------------------Let's make a discretized C space for using an A* like algorithm on--------------------

	int cspace_length = ceil(length_of_direction_vector / cspace_resolution);

	//cout << "cspace_length: " << cspace_length << endl;

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

	double xzdistance = length_of_direction_vector;
	double xzsintheta = direction[2] / xzdistance;
	double xzcostheta = xydistance / xzdistance;

	double new_y, new_x, new_z, newer_x;

	int obstacle_point_center_x, obstacle_point_center_y, obstacle_point_center_z;
	int corner_point_y, corner_point_z;

	int count_points_within = 0;

	int y_itr, x_itr, z_itr;
	//cout << "in isReachable function" << endl;

	// Find bounding box shitt.. 

	float robot_bbx_corner [3] , goal_bbx_corner[3];
	
	//2*(signbit(direction[0]) - 0.5) * robot_diameter;
	/*robot_bbx_corner [0] = - ((direction[0] > 0) - (direction[0] < 0)) * robot_diameter +robot_position[0];
	robot_bbx_corner [1] = - ((direction[1] > 0) - (direction[1] < 0)) * robot_diameter + robot_position[1];
	robot_bbx_corner [2] = - ((direction[2] > 0) - (direction[2] < 0)) * robot_diameter + robot_position[2];
	goal_bbx_corner [0] = ((direction[0] > 0) - (direction[0] < 0)) * robot_diameter + direction[0] + robot_position[0];
	goal_bbx_corner [1] = ((direction[1] > 0) - (direction[1] < 0)) * robot_diameter + direction[1] + robot_position[1];
	goal_bbx_corner [2] = ((direction[2] > 0) - (direction[2] < 0)) * robot_diameter + direction[2] + robot_position[2];*/
	robot_bbx_corner [0] = - ((direction[0] > 0) - (direction[0] < 0)) * robot_diameter +robot_sphere_matrix_position[0];
	robot_bbx_corner [1] = - ((direction[1] > 0) - (direction[1] < 0)) * robot_diameter + robot_sphere_matrix_position[1];
	robot_bbx_corner [2] = - ((direction[2] > 0) - (direction[2] < 0)) * robot_diameter + robot_sphere_matrix_position[2];
	goal_bbx_corner [0] = ((direction[0] > 0) - (direction[0] < 0)) * robot_diameter + direction[0] + robot_sphere_matrix_position[0];
	goal_bbx_corner [1] = ((direction[1] > 0) - (direction[1] < 0)) * robot_diameter + direction[1] + robot_sphere_matrix_position[1];
	goal_bbx_corner [2] = ((direction[2] > 0) - (direction[2] < 0)) * robot_diameter + direction[2] + robot_sphere_matrix_position[2];
	for(int i = 0; i < 3; i++){
		if(robot_bbx_corner[i] == 0){
			robot_bbx_corner[i] = - robot_diameter;
			goal_bbx_corner[i] = robot_diameter + direction[i];
		}
	}

	
	point3d min_point;
	min_point.x() = min(robot_bbx_corner [0], goal_bbx_corner [0]); min_point.y() = min(robot_bbx_corner [1], goal_bbx_corner [1]); 
	min_point.z() = min(robot_bbx_corner [2], goal_bbx_corner [2]);
    //cout << "min: " << min_point << endl;
    point3d max_point; 
    max_point.x() = max(robot_bbx_corner [0], goal_bbx_corner [0]); max_point.y() = max(robot_bbx_corner [1], goal_bbx_corner [1]);
    max_point.z() =max(robot_bbx_corner [2], goal_bbx_corner [2]);
    //cout << "max: " << max_point << endl;

    for(OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min_point,max_point),end=tree->end_leafs_bbx(); it!= end; ++it)
    {
    	if(it->getValue() > 0 ){
            // Node is occupied 
    		// check if within diameter distance..
    		//x=it.getX() - robot_position[0]; y=it.getY() - robot_position[1]; z=it.getZ() - robot_position[2];
    		x=it.getX() - robot_sphere_matrix_position[0]; y=it.getY() - robot_sphere_matrix_position[1]; z=it.getZ() - robot_sphere_matrix_position[2];
    		
    		iswithin = CylTest_CapsFirst(direction, length_of_direction_vector_robot_width_sq, diameter_sq, x,y,z);


    		if(iswithin>0){
				count_points_within++;

				//vector <double> newColumn;
				//newColumn.push_back(x);
				//newColumn.push_back(y);
				//newColumn.push_back(z);
				//points_within_matr.push_back(newColumn);

				// Now, if we are going to work with C space, we should find the location of the obstacle points
				// with respect to the direction vector. i.e. we should map them as if the direction vector was the x (forward) axis.

				//so we first account for rotation in xy plane:
				new_x = x * xycostheta + y * xysintheta;
				new_y = -x * xysintheta + y * xycostheta;
				//then account for rotation in the xz plane: 
				newer_x = new_x * xzcostheta + z * xzsintheta;
				new_z = - new_x * xzsintheta + z * xzcostheta;

				/*vector <double> newColumn2;
				newColumn2.push_back(newer_x);
				newColumn2.push_back(new_y);
				newColumn2.push_back(new_z);
				obstacle_points_direction_frame.push_back(newColumn2);*/

                                //cout << "point: " << newer_x << " " << new_y << " " << new_z << " ";

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

				//profum thennan stutta koda i stadinn til ad athuga hvort hann virki:
				for(county = 0; county< cspace_half_width ; county ++){
                    for(countz = 0; countz< cspace_half_width ; countz ++){
						for(countx = 0; countx< cspace_half_width ; countx ++){
							if(sphere_model[countx][county][countz] == 1){

								if(obstacle_point_center_x+countx<cspace_length && obstacle_point_center_x+countx > 0){
									if(obstacle_point_center_y+county<cspace_width && obstacle_point_center_y+county >=0){
										if(obstacle_point_center_z+countz<cspace_width && obstacle_point_center_z+countz >=0){
											Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
										}
										if(obstacle_point_center_z-countz >=0 && obstacle_point_center_z-countz<cspace_width){
											Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
										}
									}
									if(obstacle_point_center_y-county >=0 && obstacle_point_center_y-county<cspace_width){
										if(obstacle_point_center_z+countz<cspace_width && obstacle_point_center_z+countz >=0){
											Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
										}
										if(obstacle_point_center_z-countz >=0 && obstacle_point_center_z-countz<cspace_width){
											Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
										}
									}

								}
								if(obstacle_point_center_x- countx>0 && obstacle_point_center_x-countx<cspace_length){
									if(obstacle_point_center_y+county<cspace_width && obstacle_point_center_y+county >=0){
										if(obstacle_point_center_z+countz<cspace_width && obstacle_point_center_z+countz >=0){
											Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
										}
										if(obstacle_point_center_z-countz >=0 && obstacle_point_center_z-countz<cspace_width){
											Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
										}
									}
									if(obstacle_point_center_y-county >=0 && obstacle_point_center_y-county<cspace_width){
										if(obstacle_point_center_z+countz<cspace_width && obstacle_point_center_z+countz >=0){
											Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
										}
										if(obstacle_point_center_z-countz >=0 && obstacle_point_center_z-countz<cspace_width){
											Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
										}
									}
								}
							}
						}
					}
				}

				//cout << "x: " << obstacle_point_center_x << "y: " << obstacle_point_center_y << "z: " << obstacle_point_center_z << endl;
				/*
				if (obstacle_point_center_x + cspace_half_width < cspace_length && obstacle_point_center_x - cspace_half_width > 0 ){
                    if(obstacle_point_center_y < 0){
						// This means the center is to the left of the matrix
						// so we only have to fill up the right part of the sphere
						if (obstacle_point_center_z < 0){
							// Obstacle point below matrix
							// so only fill up upper right half of sphere

							// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
							// So the point we want to get to is:

							//technically we are dealing with a corner line, but it is a point in the yz plane
							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for( y_itr = 0; y_itr < corner_point_y; y_itr++){
								for( z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for( x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][-obstacle_point_center_z+z_itr] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
										}
									countx++;
									}
								}
							}
						}
						else if (obstacle_point_center_z >= cspace_width){
							// Obstacle point above matrix
							// only fill in bottom right half of sphere

							// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
							// So the point we want to get to is:

							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for(y_itr = 0; y_itr < corner_point_y; y_itr++){
								countz =0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for( x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][obstacle_point_center_z-cspace_width+countz] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
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

							if(obstacle_point_center_z< cspace_half_width){
								// this means we are below middle
								// so we need to continuously check if the lower part of our sphere is out of bounds

								//technically we are dealing with a corner line, but it is a point in the yz plane
								corner_point_z = obstacle_point_center_z + cspace_half_width;
								corner_point_y = obstacle_point_center_y + cspace_half_width;

								for(y_itr = 0; y_itr < corner_point_y; y_itr++){
									countz = 0;
									for(z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
										countx = 0;
										for( x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z - countz >= 0){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z - countz]=1;
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

								for(y_itr = 0; y_itr < corner_point_y; y_itr++){
									countz =0;
									for(z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
										// we also have to iterate over this in x direction.. 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){ 
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z + countz < cspace_width){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z + countz] = 1;
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
						if (obstacle_point_center_z < 0){
							// Obstacle point below matrix
							// so only fill up upper left half of sphere

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][-obstacle_point_center_z+z_itr] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											Cspace[obstacle_point_center_x - countx][y_itr][z_itr]=1;
										}
									countx++;
									}
								}
								county++;
							}
						}
						else if (obstacle_point_center_z > cspace_width){
							// Obstacle point above matrix
							// only fill in bottom left half of sphere
							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;
							county = 0;
							for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								countz =0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][obstacle_point_center_z-cspace_width+countz] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
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

								//technically we are dealing with a corner line, but it is a point in the yz plane
								corner_point_z = obstacle_point_center_z + cspace_half_width;
								corner_point_y = obstacle_point_center_y - cspace_half_width;
								county = 0;
								for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
									countz = 0;
									for(z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z - countz >= 0){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z - countz]=1;
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

								corner_point_z = obstacle_point_center_z - cspace_half_width;
								corner_point_y = obstacle_point_center_y - cspace_half_width;
								county = 0;
								for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
									countz =0;
									for(z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
										// we also have to iterate over this in x direction.. 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){ 
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
												if(obstacle_point_center_z + countz < cspace_width){
													Cspace[obstacle_point_center_x- countx][y_itr][obstacle_point_center_z + countz] = 1;
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

						//well now, this is getting (unnecessary) lenghty, for only a little bit faster program... 
						// Let's check if we are to the right or left of the center:
						if(obstacle_point_center_y< cspace_half_width){
							// this means we are left of center, below matrix
							// need to monitor that left part of sphere does not go out of bounds

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++ ){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y - county >= 0){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y - county][z_itr]=1;
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

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
											
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;

											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y + county < cspace_width){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y + county][z_itr]=1;
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

						//ugh, let's check if we are to right or left of middle point.. 
						if(obstacle_point_center_y< cspace_half_width){
							// this means we are left of center
							// need to monitor that left part of sphere does not go out of bounds
							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
								countz = 0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr -- ){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][countz] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y - county >= 0){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y - county][z_itr]=1;
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

							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
								countz = 0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr--){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][countz] == 1){
											Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;
											Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											if(obstacle_point_center_y + county < cspace_width){
												Cspace[obstacle_point_center_x- countx][obstacle_point_center_y + county][z_itr]=1;
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

						// gerum bara einfaldan koda fyrir thetta til ad byrja med, er ordinn ansi threyttur a thessu...

						for(county = 0; county< cspace_half_width ; county ++){
                            for(countz = 0; countz< cspace_half_width; countz ++){
								for(countx = 0; countx< cspace_half_width ; countx ++){

									if(sphere_model[countx][county][countz] == 1){

                                        if(obstacle_point_center_y+county<cspace_width){
                                                if(obstacle_point_center_z+countz<cspace_width){ 
                                                        Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
                                                        Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z+countz] =1;
                                                }
                                                if(obstacle_point_center_z-countz >=0){
                                                        Cspace[obstacle_point_center_x+countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
                                                        Cspace[obstacle_point_center_x-countx][obstacle_point_center_y+county][obstacle_point_center_z-countz] =1;
                                                }
                                        }
                                        if(obstacle_point_center_y-county >=0){
                                                if(obstacle_point_center_z+countz<cspace_width){
                                                        Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
                                                        Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z+countz] =1;
                                                }
                                                if(obstacle_point_center_z-countz >=0){
                                                        Cspace[obstacle_point_center_x+countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
                                                        Cspace[obstacle_point_center_x-countx][obstacle_point_center_y-county][obstacle_point_center_z-countz] =1;
                                                }
                                        }
									}
								}
							}
						}
					}
				}
				else{ //(obstacle_point_center_x < cspace_length){ //&& obstacle_point_center_x >=0){
					if(obstacle_point_center_y < 0){
						// This means the center is to the left of the matrix
						// so we only have to fill up the right part of the sphere
						if (obstacle_point_center_z < 0){
							// Obstacle point below matrix
							// so only fill up upper right half of sphere

							// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
							// So the point we want to get to is:

							//technically we are dealing with a corner line, but it is a point in the yz plane
							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for(y_itr = 0; y_itr < corner_point_y; y_itr++){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][-obstacle_point_center_z+z_itr] == 1){
											if(x_itr < cspace_length && x_itr >=0) Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											} 
										}
									countx++;
									}
								}
							}
						}
						else if (obstacle_point_center_z > cspace_width){
							// Obstacle point above matrix
							// only fill in bottom right half of sphere

							// Perhaps we can just iterate over the values that are inside the obstacle matrix... 
							// So the point we want to get to is:

							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							for(y_itr = 0; y_itr < corner_point_y; y_itr++){
								countz =0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][-obstacle_point_center_y+y_itr][obstacle_point_center_z-cspace_width+countz] == 1){
											if(x_itr < cspace_length && x_itr >=0) Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
											} 
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

							if(obstacle_point_center_z< cspace_half_width){
								// this means we are below middle

                                                            // I guess we also have to check if the sphere goes out of bounds on either side length wise

                                                            //we can then iterate from the bottom left half of the matrix...

								//technically we are dealing with a corner line, but it is a point in the yz plane
								corner_point_z = obstacle_point_center_z + cspace_half_width;
								corner_point_y = obstacle_point_center_y + cspace_half_width;

								for(y_itr = 0; y_itr < corner_point_y; y_itr++){
									countz = 0;
									for(z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){
												if(x_itr < cspace_length && x_itr >=0){
													Cspace[x_itr][y_itr][z_itr] = 1;
													if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
												} 
												if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

								for(y_itr = 0; y_itr < corner_point_y; y_itr++){
									countz =0;
									for(z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
										// we also have to iterate over this in x direction.. 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][-obstacle_point_center_y+y_itr][countz] == 1){
												if(x_itr < cspace_length && x_itr >=0){ 
													Cspace[x_itr][y_itr][z_itr] = 1;
													if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
												}
												if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){ 
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
						if (obstacle_point_center_z < 0){
							// Obstacle point below matrix
							// so only fill up upper left half of sphere

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++ ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(int x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][-obstacle_point_center_z+z_itr] == 1){
											if(x_itr < cspace_length && x_itr >=0) Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
												Cspace[obstacle_point_center_x - countx][y_itr][z_itr]=1;
											}
										}
									countx++;
									}
								}
								county++;
							}
						}
						else if (obstacle_point_center_z > cspace_width){
							// Obstacle point above matrix
							// only fill in bottom left half of sphere
							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;
							county = 0;
							for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
								countz =0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr-- ){
									// we also have to iterate over this in x direction.. 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][obstacle_point_center_z-cspace_width+countz] == 1){
											if(x_itr < cspace_length && x_itr >=0) Cspace[x_itr][y_itr][z_itr] = 1;
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length) {
												Cspace[obstacle_point_center_x- countx][y_itr][z_itr]=1;
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
							// fill in the whole left half
							// Athugum hvort vid seum fyrir ofan eda nedan midju:

							if(obstacle_point_center_z< cspace_half_width ){
								// this means we are below middle
								// so we need to continuously check if the lower part of our sphere is out of bounds

								//technically we are dealing with a corner line, but it is a point in the yz plane
								corner_point_z = obstacle_point_center_z + cspace_half_width;
								corner_point_y = obstacle_point_center_y - cspace_half_width;
								county = 0;
								for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
									countz = 0;
									for(z_itr = obstacle_point_center_z; z_itr < corner_point_z; z_itr ++ ){ 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){
												if(x_itr < cspace_length && x_itr >=0){
													Cspace[x_itr][y_itr][z_itr] = 1;
													if(obstacle_point_center_z - countz >= 0) Cspace[x_itr][y_itr][obstacle_point_center_z - countz] = 1;
												} 
												if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

								corner_point_z = obstacle_point_center_z - cspace_half_width;
								corner_point_y = obstacle_point_center_y - cspace_half_width;
								county = 0;
								for(y_itr = cspace_width-1; y_itr > corner_point_y; y_itr--){
									countz =0;
									for(z_itr = obstacle_point_center_z; z_itr > corner_point_z; z_itr-- ){
										// we also have to iterate over this in x direction.. 
										countx = 0;
										for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
											if(sphere_model[countx][obstacle_point_center_y- cspace_width +county][countz] == 1){
												if(x_itr < cspace_length && x_itr >=0){ 
													Cspace[x_itr][y_itr][z_itr] = 1;
													if(obstacle_point_center_z + countz < cspace_width) Cspace[x_itr][y_itr][obstacle_point_center_z + countz] = 1;
												}
												if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){ 
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

						//well now, this is getting (unnecessary) lenghty, for only a little bit faster program... 
						// Let's check if we are to the right or left of the center:
						if(obstacle_point_center_y< cspace_half_width){
							// this means we are left of center, below matrix
							// need to monitor that left part of sphere does not go out of bounds

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++ ){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
											if(x_itr < cspace_length && x_itr >=0){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
											} 
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

							corner_point_z = obstacle_point_center_z + cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
								for(z_itr = 0; z_itr < corner_point_z; z_itr ++){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][-obstacle_point_center_z+z_itr] == 1){
											if(x_itr < cspace_length && x_itr >=0){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;
											} 
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

						//ugh, let's check if we are to right or left of middle point.. 
						if(obstacle_point_center_y< cspace_half_width){
							// this means we are left of center
							// need to monitor that left part of sphere does not go out of bounds
							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y + cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr < corner_point_y; y_itr++){
								countz = 0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr -- ){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x ;x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][countz] == 1){
											if(x_itr < cspace_length && x_itr >=0){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_y - county >= 0) Cspace[x_itr][obstacle_point_center_y - county][z_itr] = 1;
											} 
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

							corner_point_z = obstacle_point_center_z - cspace_half_width;
							corner_point_y = obstacle_point_center_y - cspace_half_width;

							county = 0;
							for(y_itr = obstacle_point_center_y; y_itr > corner_point_y; y_itr--){
								countz = 0;
								for(z_itr = cspace_width-1; z_itr > corner_point_z; z_itr--){ 
									countx = 0;
									for(x_itr = obstacle_point_center_x; x_itr < obstacle_point_center_x + cspace_half_width ;x_itr++){
										if(sphere_model[countx][county][countz] == 1){
											if(x_itr < cspace_length && x_itr >=0){
												Cspace[x_itr][y_itr][z_itr] = 1;
												if(obstacle_point_center_y + county < cspace_width) Cspace[x_itr][obstacle_point_center_y + county][z_itr] = 1;
											} 
											if(obstacle_point_center_x- countx >0 && obstacle_point_center_x- countx <cspace_length){
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

						// gerum bara einfaldan koda fyrir thetta til ad byrja med, er ordinn ansi threyttur a thessu...

						for(county = 0; county< cspace_half_width ; county ++){
                            for(countz = 0; countz< cspace_half_width ; countz ++){
								for(countx = 0; countx< cspace_half_width ; countx ++){
									if(sphere_model[countx][county][countz] == 1){

										if(obstacle_point_center_x+countx<cspace_length && obstacle_point_center_x+countx>=0){
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
										if(obstacle_point_center_x- countx>0 && obstacle_point_center_x- countx <cspace_length){
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
					} 
				} //her endar thessi langa ef setning  */
			}
    	}
    }



	//cout << "count_points_within : " << count_points_within << endl;

		//-------------------Transform again to point cloud for visualizing in rviz:------------------------
        // /*
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
        // /*
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
        // */

  	//------------------------------------Now use graph search to find if subgoal is reachable:--------------------------------------

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
		  	for (y_itr = 0; y_itr < cspace_width; y_itr++){
		  		for (z_itr=0; z_itr<cspace_width; z_itr++){
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
		  		if(front_edge <= 0){
		  			// now we are back where we started soo..
		  			cout << "subgoal not reachable" << endl;
                    return false;
                    break;
		  		}
		  		countx = 0;
		  		for (y_itr = 0; y_itr < cspace_width; y_itr++){
		  			for (z_itr=0; z_itr<cspace_width; z_itr++){
		  				if(Cspace[front_edge][y_itr][z_itr] == 2) countx += 1;
		  			}
		  		}
		  		if(countx == Cspaceframe_activepoints){
		  			// this means we are going backwards but all points in cross section are reachable.. 
		  			//cout << "not reachable" << endl;
		  			return false;
		  			break;
		  			//condition =false;
                }
                //cout << " front edge is: " << front_edge ;
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
			  		for (y_itr = 0; y_itr < cspace_width; y_itr++){
				  		for (z_itr=0; z_itr<cspace_width; z_itr++){
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
			    for (y_itr = 0; y_itr < cspace_width; y_itr++){
				  	for (z_itr=0; z_itr<cspace_width; z_itr++){
				  		// check if point is in edge points:
						//if(edge_points[front_edge][y_itr][z_itr] == 1){
				  		if(Cspace[front_edge][y_itr][z_itr] != 1){ // means C space is not occupied in this point.. 
				  			countx++;
						}
					}
                }
                cout << "count is: "<< countx <<  endl;
			    if(countx == Cspaceframe_activepoints){
			    	return true;
			    }
                else{
                    cout << "subgoal not reachable" << endl;
                    return false;
                }

                break;
                //condition =false;
            }
        }
	}

}