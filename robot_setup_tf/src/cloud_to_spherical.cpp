#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
//maybe use this: ?
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <cmath>
#include "math.h"




ros::Publisher pub;

//using namespace cmath
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);

	double x;
	double y;
	double z;
	double r, rho, phi, theta;
	double pi = 3.14159265359;

  // If we use boxes with 3 degrees width and height, there are m=60 * n=120 sections
  /*sphere_matrix = new int*[61];
  for(int lin=0;lin<61;lin++)
  {
  	sphere_matrix[lin] = new int[120];
  }*/

	double sphere_matrix [60][120] = {0};

	//For some reason, for the camera in the camera_link frame, z is forward, y down and x to the right. 
	// weird.
	//cout << "in function" << endl;
	int m,n;
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
			m = floor((phi + pi/2) * 60 / pi);
			n = floor((theta + pi) * 60 / pi);
			
			//cout << "whatsup!" << endl;
			//cout << "m:" << m << "n:" << n << endl;

			if(m> -1 && n> -1 && m < 60 && n< 120){ //~isnan(m) && ~isnan(n)){
				if( rho < sphere_matrix[m][n] || sphere_matrix[m][n] ==0){
					//cout << "in" << endl;
					sphere_matrix[m][n] = rho;	
			}
			}
			else{
				cout << "m: " << m << " n:" << n << endl;
			}
		}
	}

	//cout <<"2" << endl;
  //transform again to point cloud for visualizing in rviz:

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

}

int
main (int argc, char** argv)
{
  // Initialize ROS
	ros::init (argc, argv, "cloud_to_spherical");
	ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);//("/camera/depth/points", 1, cloud_cb);//

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("spherical_cloud", 1);

  // Spin
  ros::spin ();
}