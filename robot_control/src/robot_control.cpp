// template from:      https://github.com/wsnewman/learning_ros/tree/master/Part_1/example_ros_class/src

// NODE FOR HIGH LEVEL CONTROL OF DRONE

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "robot_control.h"

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
    // robot_radius = 0.35;//0.4;
    // robot_diameter = 0.7;//0.8;
    robot_radius = 0.4;
    robot_diameter = 0.8;
    // This is velocity safety distance:
    velocity_safety_distance = 1.2;

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
    v_max = 0.4; // meters per second maximum translational setpoint_velocity     0.2    0.13
    omega_max = pi/6;//pi/4; // rad/sec maximum angular                            pi/6  pi/10

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

    fixed_position[0] = 0;
    fixed_position[1] = 0;
    fixed_position[2] = 1;

    // can also do tests/wait to make sure all required services, topics, etc are alive
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

    goal_position_sub_ = nh_.subscribe("/goal_position", 1, &RobotControl::goalPositionCallback, this);
    //direction_vector_sub_ = nh_.subscribe("/target_position", 1, &RobotControl::directionVectorCallback, this);
    
    //u_sol_sub_ = nh_.subscribe("/u_sol", 1, &RobotControl::usolCallback, this);
    //sonar_limits_and_ranges_sub_ = nh_.subscribe("/sonar_degree_limits", 1, &RobotControl::sonarlimitsrangesCallback, this);
    //camera_subscriber_ = nh_.subscribe("/voxelpoints", 1, &RobotControl::cameraCallback, this);
    target_position_sub_ = nh_.subscribe("/target_position", 1, &RobotControl::targetPositionCallback, this);

    robot_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &RobotControl::robotPositionCallback, this);

    robot_local_velocity_sub = nh_.subscribe("/mavros/local_position/odom", 1, &RobotControl::robotLocalVelocityCallback, this);

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
    //pub_u_sol_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/u_sol_cloud", 1, true);

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
    cout << "nearest_obstacle_distance: " << nearest_obstacle_distance << endl; 

}

/*void RobotControl::directionVectorCallback(const geometry_msgs::Vector3ConstPtr& input) {

    direction_vector[0] = input->x;
    direction_vector[1] = input->y;
    direction_vector[2] = input->z;

    goal_point = true;
    //cout<<"here 1"<< endl;
    //cout << "direction_vector: " << direction_vector << endl;
}*/

void RobotControl::goalPositionCallback(const geometry_msgs::Vector3ConstPtr& input) {

	//cout << "in goal position callback " << endl;

	goal_position[0] = input->x;
	goal_position[1] = input->y;
	goal_position[2] = input->z;
    if(!goal_point){
        start_time =ros::Time::now().toSec();
        goal_point = true;
    }
	
}

void RobotControl::robotLocalVelocityCallback(const nav_msgs::OdometryConstPtr& input) {

    local_robot_translational_velocity[0] = input->twist.twist.linear.x;
    local_robot_translational_velocity[1] = input->twist.twist.linear.y;
    local_robot_translational_velocity[2] = input->twist.twist.linear.z;

    // local_robot_angular_velocity[0] = input->twist.twist.angular.x;
    // local_robot_angular_velocity[1] = input->twist.twist.angular.y;
    // local_robot_angular_velocity[2] = input->twist.twist.angular.z;
}

/*void RobotControl::usolCallback(const geometry_msgs::Vector3ConstPtr& input){

    u_sol[0] = input->x;
    u_sol[1] = input->y;
    u_sol[2] = input->z;

    // consider sending rather a goal point, so if it takes a long time to calculate the movement of the robot it won't suffer... 
    // or the length of the solution vector and calculate goal point here... 

    if(u_sol[0] < 999 ){
        orm_control = true;
        initial_mode = false; 
        Vector3d robot_orientation_v(robot_orientation[0], robot_orientation[1], robot_orientation[2]);
	    float robot_orientation_w = robot_orientation[3];

	    u_sol_global = u_sol + 2*robot_orientation_w*cross(robot_orientation_v, u_sol) +
	    2*cross(robot_orientation_v, cross(robot_orientation_v, u_sol));

	    target_position[0] = robot_position[0] + u_sol_global[0];
	    target_position[1] = robot_position[1] + u_sol_global[1];
	    target_position[2] = robot_position[2] + u_sol_global[2];   

        // Transform to point cloud to visualize in rviz:
	    PointCloud::Ptr msg (new PointCloud);
	    msg->header.frame_id = "base_link";
	    msg->height = 1;
	    msg->width = 1;

	    msg->points.push_back (pcl::PointXYZ(u_sol[0], u_sol[1], u_sol[2]));

	    // Convert to ROS data type
	    sensor_msgs::PointCloud2 output;
	    pcl::toROSMsg(*msg, output);
	    // Publish the data
	    pub_u_sol_cloud.publish (output);
    }
    else if(orm_control){
        // this happens only once when we are changing from orm control to non-orm control.
        fixed_position = robot_position;
        orm_control = false;
    }
}*/
void RobotControl::targetPositionCallback(const geometry_msgs::Vector3ConstPtr& input){

	target_position[0] = input->x;
    target_position[1] = input->y;
    target_position[2] = input->z;

    // consider sending rather a goal point, so if it takes a long time to calculate the movement of the robot it won't suffer... 
    // or the length of the solution vector and calculate goal point here... 

    if(target_position[0] < 990  && robot_position[2] > 0.1){
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

    	// Direction vector in global frame:
    	direction_vector = target_position - robot_position;

    	double length_of_direction_vector = vectorlength(direction_vector);

    	Vector3d goal_vector = goal_position - robot_position;

        double distance_from_goal_point_squared = pow(goal_vector[0], 2) + 
        pow(goal_vector[1], 2) + pow(goal_vector[2], 2); 


        // if close enough to goal point just stay there .. 
        if(distance_from_goal_point_squared < 0.09) {// if less than 30 cm from goal point...
            
            cout << "goal reached!" << endl;
            
            if(!goal_reached){
                fixed_position = robot_position;

            	q[0] = robot_orientation[0];
            	q[1] = robot_orientation[1];
            	q[2] = robot_orientation[2];
            	q[3] = robot_orientation[3];
            	tf2::Matrix3x3 m(q);
  				
  				m.getRPY(roll, pitch, goal_yaw);
  				cout << "Goal yaw : " << goal_yaw  << "!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;
                goal_reached = true;
            }

            // When using velocity commands:
            

            set_velocity.header.frame_id = "base_link";
            set_velocity.header.stamp = ros::Time::now();

            q[0] = robot_orientation[0];
        	q[1] = robot_orientation[1];
        	q[2] = robot_orientation[2];
        	q[3] = robot_orientation[3];
        	tf2::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);



            set_velocity.twist.linear.x = goal_position[0] - robot_position[0];
            set_velocity.twist.linear.y = goal_position[1] - robot_position[1];
            set_velocity.twist.linear.z = goal_position[2] - robot_position[2];

            set_velocity.twist.angular.x = 0;
            set_velocity.twist.angular.y = 0;
            //cout << "goal yaw: " << goal_yaw;

            set_velocity.twist.angular.z = (goal_yaw - yaw) * omega_max;

            // When using pose commands:
            /*set_pose.header.frame_id = "map";
            set_pose.header.stamp = ros::Time::now();
            set_pose.pose.position.x = goal_position[0]; //robot_position[0] + direction_vector[0];
            set_pose.pose.position.y = goal_position[1]; //robot_position[1] + direction_vector[1];
            set_pose.pose.position.z = goal_position[2]; //robot_position[2] + direction_vector[2];

            q2.setRPY(0, 0, goal_yaw);

            set_pose.pose.orientation.x = q2.x();
            set_pose.pose.orientation.y = q2.y();
            set_pose.pose.orientation.z = q2.z();
            set_pose.pose.orientation.w = q2.w();

            pub_desired_position_.publish(set_pose);*/

        }
        else{
            goal_reached = false;

            

            if((orm_control && (abs(direction_vector[0])+abs(direction_vector[1])+abs(direction_vector[2]) > 0.2 || 
                abs(direction_vector[2] / (abs(direction_vector[0])+abs(direction_vector[1])+abs(direction_vector[2]))) > 0.85)) && ros::Time::now().toSec() - start_time > 4){
                // do the orm control
                //cout << "in orm control" << endl;
                
                // okay let's try flying directly forward, it can be up or down, but not to the sides...
                // and just turn the robot...

                //xy_length_of_u_sol = sqrt(pow(u_sol[0],2) + pow(u_sol[1],2));

            	// theta needs to be the difference of the angle of robot and the target angle in xy plane...
                // u sol is in robot frame...
                double yaw_desired = atan2(direction_vector[1], direction_vector[0]);
                if(abs(direction_vector[0] / (abs(direction_vector[0])+abs(direction_vector[1])+abs(direction_vector[2]))) > 0.95 && direction_vector[0] < 0) yaw_desired = pi;

                q[0] = robot_orientation[0];
            	q[1] = robot_orientation[1];
            	q[2] = robot_orientation[2];
            	q[3] = robot_orientation[3];
            	tf2::Matrix3x3 m(q);
  				double roll, pitch, yaw;
  				m.getRPY(roll, pitch, yaw);

  				

                theta = yaw_desired - yaw;//atan2(u_sol[1], u_sol[0]);

                //cout << "yaw: "<<yaw << "yaw_desired: " << yaw_desired << "theta: " << theta <<endl;

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
                
                if(theta < -pi) theta  += 2*pi;
                else if(theta > pi) theta -= 2*pi;


               	if(nearest_obstacle_distance < robot_radius + 0.02 ){ // + 0.05
               		// very close to obstacle, fly in opposite direction.. 
               		set_velocity.header.frame_id = "base_link";
                    set_velocity.header.stamp = ros::Time::now();
                    

                    double theta_close_obstacle, phi_close_obstacle, r_close_obstacle;


                    if(robot_position[2] < 0.3){
                        v=v_max;
                    }
                    else{
                        v = (robot_radius + 0.02 - nearest_obstacle_distance)*0.3 +0.02; //+0.01   
                    }
                    
               		theta_close_obstacle = nearest_obstacle_distance_n *  pi / M - pi;
					phi_close_obstacle = nearest_obstacle_distance_m * pi / M - pi/2;
					r_close_obstacle = cos(phi_close_obstacle);

					u_sol[0] = -v*r_close_obstacle * cos(theta_close_obstacle);
					u_sol[1] = -v*r_close_obstacle * sin(theta_close_obstacle);
					u_sol[2] = -v*sin(phi_close_obstacle);

			        Vector3d robot_orientation_v(robot_orientation[0], robot_orientation[1], robot_orientation[2]);
				    float robot_orientation_w = robot_orientation[3];

				    u_sol_global = u_sol + 2*robot_orientation_w*cross(robot_orientation_v, u_sol) +
				    2*cross(robot_orientation_v, cross(robot_orientation_v, u_sol));

					set_velocity.twist.linear.x = u_sol_global[0];
					set_velocity.twist.linear.y = u_sol_global[1];
					set_velocity.twist.linear.z = u_sol_global[2];

					set_velocity.twist.angular.x = 0;
                    set_velocity.twist.angular.y = 0;
                    set_velocity.twist.angular.z = 0;

                    cout << "very close to obstacle, go in opposite direction " << endl; 

               	}
               	else{
	                //cout << "theta in orm: " << theta << endl;
	                if(abs(theta) > 10 * pi / 180 && abs(direction_vector[2] / (abs(direction_vector[0])+abs(direction_vector[1])+abs(direction_vector[2]))) < 0.85){
	                    // turn the robot..
                        // JUST COMMENTED THIS OUT:....
	                    if(!orm_turn_mode){
                            if(pow(local_robot_translational_velocity[0], 2) + pow(local_robot_translational_velocity[1], 2) + pow(local_robot_translational_velocity[2], 2) < 0.0004){ // if velocity less than 2cm
                                fixed_position = robot_position;
                                if(fixed_position[2] <0.5){
                                    fixed_position[2] = 0.5;
                                }
                                orm_turn_mode = true;
                            }

                            set_velocity.twist.linear.x = 0;//fixed_position[0] - robot_position[0]; try without fixed position
                            set_velocity.twist.linear.y = 0;//fixed_position[1] - robot_position[1];
                        
                            if(robot_position[2] < 0.5){
                                set_velocity.twist.linear.z = 0.5 - robot_position[2];
                            }
                            else{
                                set_velocity.twist.linear.z =0;
                            }
                            // else{
                            //     // just do zero velocity .. 
                            // }
	                        
	                    }
                        else{
                            // now follow fixed position
                            set_velocity.twist.linear.x = 0.3*(fixed_position[0] - robot_position[0]); 
                            set_velocity.twist.linear.y = 0.3*(fixed_position[1] - robot_position[1]);
                            set_velocity.twist.linear.z = 0.3*(fixed_position[2] - robot_position[2]);
                        }
	                    
	                    // if using pose command:
	                    /*set_pose.header.frame_id = "map";
	                    set_pose.header.stamp = ros::Time::now();
	                    set_pose.pose.position.x = fixed_position[0];
	                    set_pose.pose.position.y = fixed_position[1];
	                    set_pose.pose.position.z = fixed_position[2];*/

	                    /*
	                    xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));
						q[0] = robot_orientation[0];
	                	q[1] = robot_orientation[1];
	                	q[2] = robot_orientation[2];
	                	q[3] = robot_orientation[3];
	                	tf2::Matrix3x3 m(q);
	      				double roll, pitch, yaw;
	      				m.getRPY(roll, pitch, yaw);
						*/

	                    // also this for pose command:
	                    /*tf2::Quaternion q;

	                    cout << "angle in orm control: " << yaw_desired << endl;
	                    q.setRPY(0, 0, yaw_desired);

	                    set_pose.pose.orientation.x = q.x();
	                    set_pose.pose.orientation.y = q.y();
	                    set_pose.pose.orientation.z = q.z();
	                    set_pose.pose.orientation.w = q.w();


	                    /*z_turn = 1*sin(theta+yaw/2);
	                    w_turn = cos(theta+yaw/2);

	                    //if(xy_length_of_direction_vector >0.2){
	                    set_pose.pose.orientation.x = 0;
	                    set_pose.pose.orientation.y = 0;
	                    set_pose.pose.orientation.z = z_turn;
	                    set_pose.pose.orientation.w = w_turn;                    
	                    //}
	                    /*else{

	                        set_pose.pose.orientation.x = 0;
	                        set_pose.pose.orientation.y = 0;
	                        set_pose.pose.orientation.z = robot_orientation[2];
	                        set_pose.pose.orientation.w = robot_orientation[3]; 
	                    }*/

	                    // You can do something like this...
	                    /*
	                    tf2::Quaternion q;
	                    q.setRPY(0, 0, theta);

	                    set_pose.pose.orientation.x = q.x();
	                    set_pose.pose.orientation.y = q.y();
	                    set_pose.pose.orientation.z = q.z();
	                    set_pose.pose.orientation.w = q.w();
	                    */

	                    //pub_desired_position_.publish(set_pose);

	                    // just so it can turn around if its supposed to turn 180 degrees:
	                    /*if(theta > pi - 0.3 && theta < pi+0.3 || theta > -pi - 0.3 && theta < -pi+0.3){
	                    	theta = 2.5; // just to turn in one direction..
	                    	cout << "now theta is 2.5" << endl; 
	                    }*/

	                    omega = omega_max * theta / half_pi;
	                	
                        // Try one more thing, only register fixed position when robot has stopped...

                        set_velocity.twist.angular.x = 0;
                        set_velocity.twist.angular.y = 0;
                        set_velocity.twist.angular.z = omega;

                        // // Allow for some drift but not too much:
                        // double distance_from_fixed_position_sq = pow((fixed_position[0] - robot_position[0]),2) + pow((fixed_position[1] - robot_position[1]),2) + pow((fixed_position[2] - robot_position[2]),2);
                        // if(distance_from_fixed_position_sq < 0.01){// less than 10 cm away from fixed position.. 
                        //     set_velocity.twist.linear.x = 0;//fixed_position[0] - robot_position[0]; try without fixed position
                        //     set_velocity.twist.linear.y = 0;//fixed_position[1] - robot_position[1];
                        
                        //     if(robot_position[2] < 0.5){
                        //         set_velocity.twist.linear.z = 0.5 - robot_position[2];
                        //     }
                        //     else{
                        //         set_velocity.twist.linear.z =0;
                        //     }

                        // }
                        // else{
                        //     set_velocity.twist.linear.x = 0.3*(fixed_position[0] - robot_position[0]); 
                        //     set_velocity.twist.linear.y = 0.3*(fixed_position[1] - robot_position[1]);
                        //     set_velocity.twist.linear.z = 0.3*(fixed_position[2] - robot_position[2]);
    	                    
                        //     set_velocity.twist.angular.x = 0;
    	                   //  set_velocity.twist.angular.y = 0;
    	                   //  set_velocity.twist.angular.z = omega;
                        // }
	                }
	                else{

	                	// fly

	                    //geometry_msgs::TwistStamped set_velocity;
	                    set_velocity.header.frame_id = "base_link";
	                    set_velocity.header.stamp = ros::Time::now();


	                    if(abs(direction_vector[2] / (abs(direction_vector[0])+abs(direction_vector[1])+abs(direction_vector[2]))) > 0.85){
	                    	// this means we go straight down or up
                            
                            //NEW:
                            if(nearest_obstacle_distance > velocity_safety_distance || robot_position[2] < 0.15){
                                v = v_max;
                            }
                            else{
                                v = v_max * (nearest_obstacle_distance-robot_radius)/(velocity_safety_distance);
                            }

	                    	//v = v_max; OLD
	                    	

                            /*set_pose.pose.orientation.x = robot_orientation[0];
		                    set_pose.pose.orientation.y = robot_orientation[1];
		                    set_pose.pose.orientation.z = robot_orientation[2];
		                    set_pose.pose.orientation.w = robot_orientation[3];
		                    cout << "robot angle in moving control" << endl;*/
		                    set_velocity.twist.angular.x = 0;
	                    	set_velocity.twist.angular.y = 0;
	                    	set_velocity.twist.angular.z = 0;

	                    }
	                    else{

	                    	/*q.setRPY(0, 0, yaw_desired);

		                    set_pose.pose.orientation.x = q.x();
		                    set_pose.pose.orientation.y = q.y();
		                    set_pose.pose.orientation.z = q.z();
		                    set_pose.pose.orientation.w = q.w();
		                    */

		                    /*q[0] = robot_orientation[0];
		                	q[1] = robot_orientation[1];
		                	q[2] = robot_orientation[2];
		                	q[3] = robot_orientation[3];
		                	tf2::Matrix3x3 m(q);
		      				double roll, pitch, yaw;
		      				m.getRPY(roll, pitch, yaw);
		      				*/
                            //OLD:
		                    // if(nearest_obstacle_distance > safety_distance){
		                    //     v = v_max * (half_pi-abs(theta))/half_pi;
		                    // }
		                    // else{
		                    //     v = v_max * nearest_obstacle_distance/safety_distance * (half_pi-abs(theta))/half_pi;
		                    // }

                            //NEW:
                            if(nearest_obstacle_distance > velocity_safety_distance){
                                v = v_max * (half_pi-abs(theta))/half_pi;
                            }
                            else{
                                v = v_max * (nearest_obstacle_distance- robot_radius)/(velocity_safety_distance) * (half_pi-abs(theta))/half_pi;
                            }

	                        omega = omega_max * theta / half_pi;
		                    set_velocity.twist.angular.x = 0;
		                    set_velocity.twist.angular.y = 0;
		                    set_velocity.twist.angular.z = omega;
		      
		                    /*
		                    cout << "angle in moving orm control: " << yaw+theta << "yaw: " << yaw << "theta: " << theta << endl;
		                    q.setRPY(0, 0, yaw+theta);
		                    set_pose.pose.orientation.x = q.x();
		                    set_pose.pose.orientation.y = q.y();
		                    set_pose.pose.orientation.z = q.z();
		                    set_pose.pose.orientation.w = q.w();*/
		                }
	                    if(distance_from_goal_point_squared < 4){
	                        // lower speed because getting close to target
	                         v *= sqrt(distance_from_goal_point_squared)/2+0.05;
	                    }
	                    //cout << "nearest_obstacle_distance: " << nearest_obstacle_distance << endl; 
	                    //cout << "u_sol in control: " << u_sol  << endl; //<< "and v is: " << v << endl;


	                    /*set_pose.pose.position.x = target_position[0];//robot_position[0]+ u_sol_global[0]*v;
	                    set_pose.pose.position.y = target_position[1];//robot_position[1]+ u_sol_global[1]*v;
	                    set_pose.pose.position.z = target_position[2];//robot_position[2]+ u_sol_global[2]*v;
	                    pub_desired_position_.publish(set_pose);
	*/
	                    Vector3d unitary_direction_vector = direction_vector/length_of_direction_vector;
	                    set_velocity.twist.linear.x = v*unitary_direction_vector[0];//u_sol_global[0]*v;
	                    set_velocity.twist.linear.y = v*unitary_direction_vector[1];
	                    set_velocity.twist.linear.z = v*unitary_direction_vector[2];

	                    //pub_desired_velocity_.publish(set_velocity);
	                    

	                    orm_turn_mode = false;

	                    //cout << "here :>)" <<endl;
	                }
	            }


            }
            else{
                // Turn toiwards goal

            	Vector3d goal_direction = goal_position - robot_position;
            	double yaw_desired =  atan2(goal_direction[1], goal_direction[0]);

            	if(fixed_position[2] <0.5){
                        	fixed_position[2] = 0.7;
                }
                // keep on same spot and turn towards target point, NOT orm control...

                //direction_vector = goal_position - robot_position;

                //cout<<"here 2"<< endl;
                //cout << "robot_vector: " << robot_position[0] << " " << robot_position[1] <<" " << robot_position[2] << endl;
                //cout << "goal_position: " << goal_position[0] << " " << goal_position[1] <<" " << goal_position[2] << endl;

                //cout << "direction_vector: " << direction_vector[0] << " " << direction_vector[1] <<" " << direction_vector[2] << endl;

                //xy_length_of_direction_vector = sqrt(pow(direction_vector[0],2) + pow(direction_vector[1],2));

                set_pose.header.frame_id = "map";
                set_pose.header.stamp = ros::Time::now();

                /*theta = atan2(direction_vector[1], direction_vector[0]);/*acos(direction_vector[0]/xy_length_of_direction_vector);
                if(direction_vector[1] < 0){
                        theta = -theta;
                }*/

                /*cout << "theta is: " << theta << endl;
                cout << "direction_vector is" << direction_vector << endl;*/
                //and we want to turn around z axis so our quaternion coordinates become:
                //double x= 0;
                //double y = 0;
                //z_turn = 1*sin(theta/2);
                //w_turn = cos(theta/2);

                
                if(initial_mode){// only do this if we haven't started moving yet.. 
                    set_velocity.twist.linear.x = 0;
                	set_velocity.twist.linear.y = 0;
                	set_velocity.twist.linear.z = 1 - robot_position[2];   // very simple P controller, consider changing...
                    //set_pose.pose.position.x = 0;
                    //set_pose.pose.position.y = 0;
                    //set_pose.pose.position.z = 1;   
                }
                else{ // otherwise keep same position if we have no u_sol ..
                	set_velocity.twist.linear.x = 0.5*(fixed_position[0] - robot_position[0]);
                	set_velocity.twist.linear.y = 0.5*(fixed_position[1] - robot_position[1]);
                	set_velocity.twist.linear.z = 0.5*(fixed_position[2] - robot_position[2]);

                    /*set_pose.pose.position.x = fixed_position[0];
                    set_pose.pose.position.y = fixed_position[1];
                    set_pose.pose.position.z = fixed_position[2];*/
                }

                

                /*if(abs(goal_direction[2] / (abs(goal_direction[0])+abs(goal_direction[1])+abs(goal_direction[2]))) > 0.85){		//xy_length_of_direction_vector >0.2){
                	set_velocity.twist.angular.x = 0;
                    set_velocity.twist.angular.y = 0;
                    set_velocity.twist.angular.z = 0;

                	/*q.setRPY(0, 0, yaw_desired);
                    set_pose.pose.orientation.x = q.x();
                    set_pose.pose.orientation.y = q.y();
                    set_pose.pose.orientation.z = q.z();
                    set_pose.pose.orientation.w = q.w();*/                   
                //}
                //else{
                	


            	q[0] = robot_orientation[0];
            	q[1] = robot_orientation[1];
            	q[2] = robot_orientation[2];
            	q[3] = robot_orientation[3];
            	tf2::Matrix3x3 m(q);
  				double roll, pitch, yaw;
  				m.getRPY(roll, pitch, yaw);
				
				theta = yaw_desired - yaw;
				set_velocity.twist.angular.x = 0;
                set_velocity.twist.angular.y = 0;
                set_velocity.twist.angular.z =  omega_max*(theta)/half_pi;


/*                    tf2::Quaternion q2;
                q2.setRPY(0, 0, yaw);
                cout << "angle in control: " << yaw << endl;
                set_pose.pose.orientation.x = q2.x();
                set_pose.pose.orientation.y = q2.y();
                set_pose.pose.orientation.z = q2.z();
                set_pose.pose.orientation.w = q2.w();
                */
                //}


                //pub_desired_position_.publish(set_pose);

                orm_turn_mode = false;
            }
        }
        pub_desired_velocity_.publish(set_velocity);
    }
}


Vector3d RobotControl::cross(const Vector3d & vec1, const Vector3d & vec2){
    Vector3d cross_product;
    cross_product[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    cross_product[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    cross_product[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    
    return ( cross_product );
}

float RobotControl::vectorlength(const Vector3d & vec){
	
	return ( sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]) );
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
