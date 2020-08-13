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
#include "nav_msgs/Odometry.h"

void Groundtruth_callback(const rosbot_multirobot_orientation::SyncedClouds::ConstPtr& sub_msg){
  nav_msgs::Odometry odom1 = sub_msg -> pose1;
  nav_msgs::Odometry odom2 = sub_msg -> pose2;
  nav_msgs::Odometry odom3 = sub_msg -> pose3;

  Eigen::Matrix4f tf1 = unavlib::cvt::geoPose2eigen(odom1.pose.pose);
  Eigen::Matrix4f tf2 = unavlib::cvt::geoPose2eigen(odom2.pose.pose);
  Eigen::Matrix4f tf3 = unavlib::cvt::geoPose2eigen(odom3.pose.pose);

  std::cout << "-- Output of rosbot1 TF --" << std::endl;
  std::cout << tf1 << std::endl;
  std::cout << "-- Output of rosbot2 TF --" << std::endl;
  std::cout << tf2 << std::endl;
  std::cout << "-- Output of rosbot3 TF --" << std::endl;
  std::cout << tf3 << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_matrix");

  ros::NodeHandle nh;

  // Subscriber Initializations to all ROSBOT 1-3s
  ros::Subscriber sub = nh.subscribe("/rosbots/clouds", 100, Groundtruth_callback);

  // Assign Publisher object with nodehandle
  ros::spin();

  return 0;
}


// void Groundtruth1_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
//   geometry_msgs::Pose pose1 = sub_msg -> pose.pose;
//   Eigen::Matrix4f tf1 = unavlib::cvt::geoPose2eigen(pose1);
//   std::cout << "-- Output of rosbot1 TF --" << std::endl;
//   std::cout << tf1 << std::endl;
// }
// void Groundtruth2_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
//   geometry_msgs::Pose pose2 = sub_msg -> pose.pose;
//   Eigen::Matrix4f tf2 = unavlib::cvt::geoPose2eigen(pose2);
//   std::cout << "-- Output of rosbot2 TF --" << std::endl;
//   std::cout << tf2 << std::endl;
// }
// void Groundtruth3_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
//   geometry_msgs::Pose pose3 = sub_msg -> pose.pose;
//   Eigen::Matrix4f tf3 = unavlib::cvt::geoPose2eigen(pose3);
//   std::cout << "-- Output of rosbot3 TF --" << std::endl;
//   std::cout << tf3 << std::endl;
// }
