#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
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
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

ros::Publisher debug_pub;

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

	  // Input Point Cloud is pcl2
	  // Target Point Cloud is pcl1

	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.35, 0.35, 0.35);
	approximate_voxel_filter.setInputCloud(pcl2);
	approximate_voxel_filter.filter(*filtered_cloud);
	static int debug_cnt = 0;
	std::cout << debug_cnt++<<"th Filtered cloud contains " << filtered_cloud->size ()
	          << " data points from input_cloud " << std::endl;
	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt12;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt12.setTransformationEpsilon (0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt12.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt12.setResolution (1.0);
	// Setting max number of registration iterations.
	ndt12.setMaximumIterations (35);

	geometry_msgs::Pose relpose12 = sub_msg->pose12;
	Eigen::Matrix4f initial_guess_eigen = unavlib::cvt::xyzrpy2eigen(relpose12.position.x, relpose12.position.y, relpose12.position.z, 0.0, 0.0, (sub_msg->init_angle)*3.1415/180);


	ndt12.setInputSource(filtered_cloud);
	ndt12.setInputTarget(pcl1);
	pcl::PointCloud<pcl::PointXYZ> output_cloud;
	ndt12.align(output_cloud, initial_guess_eigen);

  std::cout << "Normal Distributions Transform has converged:" << ndt12.hasConverged ()
            << " score: " << ndt12.getFitnessScore () << std::endl;

	// Retrieve Final Transformation
	Eigen::Matrix4f pred_T12 = ndt12.getFinalTransformation();



	// Convert 4x4 matrix to rpy for visualization
	Eigen::VectorXf pred_xyzrpy = unavlib::cvt::eigen2xyzrpy(pred_T12);

	geometry_msgs::Pose pose12 = sub_msg->pose12;
	Eigen::Matrix4f GT_T12 = unavlib::cvt::geoPose2eigen(pose12);
	Eigen::VectorXf GT_xyzrpy = unavlib::cvt::eigen2xyzrpy(GT_T12);



	// std::cout << "Predicted | T 1->2 " << std::endl;
	// std::cout<< pred_T12 << std::endl;
	// std::cout << "GT | T 1->2 " << std::endl;
	// std::cout<< GT_T12 << std::endl;
	// std::cout << " " << std::endl;
	// std::cout << " " << std::endl;
	std::cout << "\033[1;33mPredicted | Yaw 1->2 (deg): \033[0m" << pred_xyzrpy(5)/3.14*180 << std::endl;
	std::cout << "GT | Yaw 1->2 (deg): " << GT_xyzrpy(5)/3.14*180 << std::endl;
	std::cout << "GT | Distance: " << pow(pow(GT_xyzrpy(0), 2) + pow(GT_xyzrpy(1), 2), 0.5) << std::endl;
	std::cout << "__________________" << std::endl;
	std::cout << " " << std::endl;


	std_msgs::Int32 dummy;
	dummy.data = 32;
	debug_pub.publish(dummy);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosbot_ndt");
	ros::NodeHandle nh;

	// Subscriber Initializations to all ROSBOT 1-3s
	ros::Subscriber sub1 = nh.subscribe("/rosbots/clouds_angleinitialized", 1, Pointcloud_callback);
	debug_pub = nh.advertise<std_msgs::Int32>("/rosbots/ndt_debug", 100, false);
	ros::spin();

	return 0;
}
