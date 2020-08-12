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

void Groundtruth1_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
  geometry_msgs::Pose pose1 = sub_msg -> pose.pose;
  Eigen::Matrix4f tf1 = unavlib::cvt::geoPose2eigen(pose1);
  std::cout << "-- Output of rosbot1 TF --" << std::endl;
  std::cout << tf1 << std::endl;
}
void Groundtruth2_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
  geometry_msgs::Pose pose2 = sub_msg -> pose.pose;
  Eigen::Matrix4f tf2 = unavlib::cvt::geoPose2eigen(pose2);
  std::cout << "-- Output of rosbot2 TF --" << std::endl;
  std::cout << tf2 << std::endl;
}
void Groundtruth3_callback(const nav_msgs::Odometry::ConstPtr& sub_msg){
  geometry_msgs::Pose pose3 = sub_msg -> pose.pose;
  Eigen::Matrix4f tf3 = unavlib::cvt::geoPose2eigen(pose3);
  std::cout << "-- Output of rosbot3 TF --" << std::endl;
  std::cout << tf3 << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_matrix");

  ros::NodeHandle nh;

  // Subscriber Initializations to all ROSBOT 1-3s
  ros::Subscriber sub1 = nh.subscribe("/rosbot1/ground_truth/state", 100, Groundtruth1_callback);
  ros::Subscriber sub2 = nh.subscribe("/rosbot2/ground_truth/state", 100, Groundtruth2_callback);
  ros::Subscriber sub3 = nh.subscribe("/rosbot3/ground_truth/state", 100, Groundtruth3_callback);

  // Assign Publisher object with nodehandle
  ros::spin();

  return 0;
}
