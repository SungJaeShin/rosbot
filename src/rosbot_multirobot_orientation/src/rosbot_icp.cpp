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

ros::Publisher debug_pub;
std::ofstream myfile;
std::string logfile_name;

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

	// Get initial orientation from angle histogram
	geometry_msgs::Pose relpose12 = sub_msg->pose12;
 	Eigen::Matrix4f initial_guess_eigen = unavlib::cvt::xyzrpy2eigen(relpose12.position.x, relpose12.position.y, relpose12.position.z, 0.0, 0.0, (sub_msg->init_angle)*3.1415/180);


	// Conduct ICP between 1 and 2
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp12;
	icp12.setInputSource(pcl2);
	icp12.setInputTarget(pcl1);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp12.align(Final, initial_guess_eigen);

	// Retrieve Final Transformation
	Eigen::Matrix4f pred_T12 = icp12.getFinalTransformation();
	


	// Convert 4x4 matrix to rpy for visualization
	Eigen::VectorXf pred_xyzrpy = unavlib::cvt::eigen2xyzrpy(pred_T12);

	geometry_msgs::Pose pose12 = sub_msg->pose12;

	Eigen::Matrix4f GT_T12 = unavlib::cvt::geoPose2eigen(pose12);
	Eigen::VectorXf GT_xyzrpy = unavlib::cvt::eigen2xyzrpy(GT_T12);

	std::cout << "Predicted | T 1->2 " << std::endl;
	std::cout<< pred_T12 << std::endl;
	std::cout << "GT | T 1->2 " << std::endl;
	std::cout<< GT_T12 << std::endl;
	std::cout << " " << std::endl;
	std::cout << " " << std::endl;
	std::cout << "Predicted | Yaw 1->2 (deg): " << pred_xyzrpy(5)/3.14*180 << std::endl;
	std::cout << "GT | Yaw 1->2 (deg): " << GT_xyzrpy(5)/3.14*180 << std::endl;
	std::cout << "GT | Distance: " << pow(pow(GT_xyzrpy(0), 2) + pow(GT_xyzrpy(1), 2), 0.5) << std::endl;
	std::cout << "__________________" << std::endl;
	std::cout << " " << std::endl;

	// Now write log to csv file
	myfile.open(logfile_name, std::ios_base::app);
	myfile << std::to_string(pred_xyzrpy(5)/3.14*180) + ", " + std::to_string(GT_xyzrpy(5)/3.14*180) + ", "  + std::to_string(pow(pow(GT_xyzrpy(0), 2) + pow(GT_xyzrpy(1), 2), 0.5)) <<std::endl;
	myfile.close();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosbot_icp");
	ros::NodeHandle nh;

	logfile_name = "/home/sungwon/catkin_ws/csv_log/rosbot_icp_" + std::to_string(ros::Time::now().toSec()) + ".csv";
    std::ofstream output(logfile_name);
    std::cout <<"Created logfile : " << logfile_name << std::endl;
    myfile.open(logfile_name, std::ios_base::app);
    myfile << "yaw_GT, yaw_pred";
    myfile.close();

	// Subscriber Initializations to all ROSBOT 1-3s
	ros::Subscriber sub1 = nh.subscribe("/rosbots/clouds_angleinitialized", 100, Pointcloud_callback);
	ros::spin();

	return 0;
}
