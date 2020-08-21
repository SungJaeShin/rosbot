#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <rosbot_multirobot_orientation/SyncedClouds.h>
#include <unavlib/convt.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.141592

void Pointcloud_callback(const rosbot_multirobot_orientation::SyncedClouds::ConstPtr& sub_msg)
{
	// Receive subscribed message
	sensor_msgs::LaserScan scan1 = sub_msg -> scan1;
	sensor_msgs::LaserScan scan2 = sub_msg -> scan2;
	sensor_msgs::LaserScan scan3 = sub_msg -> scan3;
  nav_msgs::Odometry odom1 = sub_msg -> pose1;
  nav_msgs::Odometry odom2 = sub_msg -> pose2;
  nav_msgs::Odometry odom3 = sub_msg -> pose3;

	// // Convert pointcloud message to pointcloud library object type
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl3(new pcl::PointCloud<pcl::PointXYZ>);
  //
	// pcl::fromROSMsg(pc1,*pcl1);
	// pcl::fromROSMsg(pc2,*pcl2);
	// pcl::fromROSMsg(pc3,*pcl3);

  // To set Robot Position
  double x1 = odom1.pose.pose.position.x;
  double y1 = odom1.pose.pose.position.y;
  double z1 = odom1.pose.pose.position.z;

  double angle_interval = (scan1.angle_increment / PI) * 180;


  for(int t = 0; t <= (360 / angle_interval); t++){

  }

  std::cout << " --- " << std::endl;
  std::cout << " position of rosbot1 in x-axis : " << x1 << std::endl;
  std::cout << " position of rosbot1 in y-axis : " << y1 << std::endl;
  std::cout << " position of rosbot1 in z-axis : " << z1 << std::endl;
  std::cout << " --- " << std::endl;
  std::cout << " angle interval of rosbot1 scan (deg) : " << (scan1.angle_increment / PI) * 180 << std::endl;
  std::cout << " angle min (deg) : " << (scan1.angle_min / PI) * 180 << std::endl;
  std::cout << " angle max (deg) : " << (scan1.angle_max / PI) * 180 << std::endl;
  std::cout << " --- " << std::endl;
  std::cout << " size of ranges : " << sizeof(scan1.ranges) << std::endl;
  std::cout << " first ranges : " << scan1.ranges[0] << std::endl;



}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ndt_rel_ori");
	ros::NodeHandle nh;

	// Subscriber Initializations to all ROSBOT 1-3s
	ros::Subscriber sub1 = nh.subscribe("/rosbots/clouds", 100, Pointcloud_callback);
	ros::spin();

	return 0;
}
