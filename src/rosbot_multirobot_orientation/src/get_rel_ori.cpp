#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include "sensor_msgs/PointCloud2.h"
#include <rosbot_multirobot_orientation/SyncedClouds.h>
#include <unavlib/convt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>



void Pointcloud_callback(const rosbot_multirobot_orientation::SyncedClouds::ConstPtr& sub_msg)
{
	// Receive subscribed message
	sensor_msgs::PointCloud2 pc1 = sub_msg->pc1;
	sensor_msgs::PointCloud2 pc2 = sub_msg->pc2;
	sensor_msgs::PointCloud2 pc3 = sub_msg->pc3;

	// Convert pointcloud message to pointcloud library object type
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl3(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(pc1,*pcl1);
	pcl::fromROSMsg(pc2,*pcl2);
	pcl::fromROSMsg(pc3,*pcl3);

	// Conduct ICP between 1 and 2
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp12;
	icp12.setInputSource(pcl2);
	icp12.setInputTarget(pcl1);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp12.align(Final);

	// Retrieve Final Transformation
	Eigen::Matrix4f T12 = icp12.getFinalTransformation();
	std::cout<< T12 << std::endl;

	// Convert 4x4 matrix to rpy for visualization
	Eigen::VectorXf xyzrpy = unavlib::cvt::eigen2xyzrpy(T12);

	std::cout << "Yaw (deg): " << xyzrpy(5)/3.14*180 << std::endl;




}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_rel_ori");
	ros::NodeHandle nh;

	// Subscriber Initializations to all ROSBOT 1-3s
	ros::Subscriber sub1 = nh.subscribe("/rosbots/clouds", 100, Pointcloud_callback);
	ros::spin();

	return 0;
}