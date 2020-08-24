#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <rosbot_multirobot_orientation/SyncedClouds.h>
#include <rosbot_multirobot_orientation/AngleMatch.h>
#include <unavlib/convt.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <string.h>

#define PI 3.141592

ros::Publisher pub;

void Pointcloud_callback(const rosbot_multirobot_orientation::SyncedClouds::ConstPtr& sub_msg)
{
  // Define message to be published
  rosbot_multirobot_orientation::SyncedClouds pub_msg;
  pub_msg = *sub_msg;

	// Receive subscribed message
  sensor_msgs::PointCloud2 pc1 = sub_msg -> pc1;
  sensor_msgs::PointCloud2 pc2 = sub_msg -> pc2;
  sensor_msgs::PointCloud2 pc3 = sub_msg -> pc3;
	sensor_msgs::LaserScan scan1 = sub_msg -> scan1;
	sensor_msgs::LaserScan scan2 = sub_msg -> scan2;
	sensor_msgs::LaserScan scan3 = sub_msg -> scan3;
  nav_msgs::Odometry odom1 = sub_msg -> pose1;
  nav_msgs::Odometry odom2 = sub_msg -> pose2;
  nav_msgs::Odometry odom3 = sub_msg -> pose3;
  geometry_msgs::Pose tf12 = sub_msg -> pose12;
  geometry_msgs::Pose tf23 = sub_msg -> pose23;
  geometry_msgs::Pose tf31 = sub_msg -> pose31;

  sensor_msgs::PointCloud2 pc2_transformed;
  sensor_msgs::LaserScan scan2_transformed;

	// Convert pointcloud message to pointcloud library object type
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl3(new pcl::PointCloud<pcl::PointXYZ>);

  // Take all information of sensor_msgs::Pointcloud2
	pcl::fromROSMsg(pc1,*pcl1);
	pcl::fromROSMsg(pc2,*pcl2);
	pcl::fromROSMsg(pc3,*pcl3);

  // Make Transformation Matrix ROSBOT1 -> ROSBOT2
  Eigen::Matrix4f T12 = unavlib::cvt::geoPose2eigen(tf12);
  Eigen::VectorXf GT_xyzrpy = unavlib::cvt::eigen2xyzrpy(T12);
  T12(0, 0) = 1;
  T12(1, 1) = 1;
  T12(2, 2) = 1;
  T12(0, 1) = 0;
  T12(0, 2) = 0;
  T12(1, 0) = 0;
  T12(1, 2) = 0;
  T12(2, 0) = 0;
  T12(2, 1) = 0;

  // Transform Coordinate looked at ROSBOT1
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*pcl2, *pcl2_transformed, T12.inverse());
  pcl::toROSMsg(*pcl2_transformed, pc2_transformed);

  std::cout << " --------------------------- " << std::endl;
  // std::cout << " To know pcl1's first x value : " << pcl1 -> points[0].x << std::endl;
  // std::cout << " To know pcl1's first y value : " << pcl1 -> points[0].y << std::endl;
  // std::cout << " --------------------------- " << std::endl;
  // std::cout << " To know pcl2's first x value : " << pcl2_transformed -> points[0].x << std::endl;
  // std::cout << " To know pcl2's first y value : " << pcl2_transformed -> points[0].y << std::endl;

  std::cout << " To know pcl1's point size : " << pcl1 -> points.size() << std::endl;
  std::cout << " To know pcl2's point size : " << pcl2_transformed -> points.size() << std::endl;

  std::cout << " --------------------------- " << std::endl;
  std::cout << " Angle Interval (deg) : " <<  360 / (double)pc1.width << std::endl;

  double angle_interval1 = 360 / (double)(pcl1 -> points.size());
  double angle_interval2 = 360 / (double)(pcl2_transformed -> points.size());

  // Make AngleMatch message
  rosbot_multirobot_orientation::AngleMatch match1;
  rosbot_multirobot_orientation::AngleMatch match2;

  // Initialized Parameter Settings
  match1.angle.push_back(0);
  match1.ranges.push_back(sqrt(pow(pcl1 -> points[0].x, 2) + pow(pcl1 -> points[0].y, 2)));
  match1.header = pc1.header;

  match2.angle.push_back(0);
  match2.ranges.push_back(sqrt(pow(pcl2_transformed -> points[0].x, 2) + pow(pcl2_transformed -> points[0].y, 2)));
  match2.header = pc2.header;

  std::cout << " --------------------------- " << std::endl;
  std::cout << " match 1 values (1) angle[0] : " << match1.angle[0] << " (2) range[0] : " << match1.ranges[0] << std::endl;
  std::cout << " match 2 values (1) angle[0] : " << match2.angle[0] << " (1) range[0] : " << match2.ranges[0] << std::endl;

  // Setting two values (angle interval and distance) in two AngleMatch message
  for(int t = 1; t <= (pcl1 -> points.size()); t++){
    float tmp_angle1 = match1.angle[t - 1] + angle_interval1;
    float tmp_range1 = sqrt(pow(pcl1 -> points[t].x , 2) + pow(pcl1 -> points[t].y, 2));
    float tmp_angle2 = match2.angle[t - 1] + angle_interval1;
    float tmp_range2 = sqrt(pow(pcl2_transformed -> points[t].x , 2) + pow(pcl2_transformed -> points[t].y, 2));

    match1.angle.push_back(tmp_angle1);
    match1.ranges.push_back(tmp_range1);
    match2.angle.push_back(tmp_angle2);
    match2.ranges.push_back(tmp_range2);
  }

  // std::cout << " -------- match1 ranges ------- " << std::endl;
  // for(int t = 0; t <= (360 / angle_interval1); t++){
  //   std::cout << match1.ranges[t] << ", ";
  // }
  // std::cout << "\n -------- match1 angle ------- " << std::endl;
  // for(int t = 0; t <= (360 / angle_interval1); t++){
  //   std::cout << match1.angle[t] << ", ";
  // }
  // std::cout << "\n -------- match2 ranges ------- " << std::endl;
  // for(int t = 0; t <= (360 / angle_interval1); t++){
  //   std::cout << match2.ranges[t] << ", ";
  // }
  // std::cout << "\n -------- match2 angle ------- " << std::endl;
  // for(int t = 0; t <= (360 / angle_interval1); t++){
  //   std::cout << match2.angle[t] << ", ";
  // }

  double GT_yaw = GT_xyzrpy(5) / (2 * PI) * 360;

  // Set parameters to store values
  double index = 0;
  double min_value;
  double angle_rotation;
  bool is_first = true;
  bool is_not_finish = true;

  double save_index;

  while(is_not_finish){
    double tmp_value = 0;

    for(int t = 0; t <= (360 / angle_interval1); t++){
      tmp_value += pow((match1.ranges[t] - match2.ranges[t]), 2);
    }

    match1.norm12.push_back(tmp_value);

    if(is_first){
      min_value = tmp_value;
      angle_rotation = 0;
      is_first = false;
    }

    if(min_value > tmp_value && min_value < 500000){
      min_value = tmp_value;
      angle_rotation = index * angle_interval1;
      save_index = index;
    }

    double tmp_range = match2.ranges[0];
    double tmp_angle = match2.angle[0];

    for(int t = 1; t <= (360 / angle_interval1); t++){
      match2.angle[t - 1] = match2.angle[t];
      match2.ranges[t - 1] = match2.ranges[t];
    }

    match2.angle[(int)(360 / angle_interval1)] = tmp_angle;
    match2.ranges[(int)(360 / angle_interval1)] = tmp_range;

    index++;
    if(index >= (360 / angle_interval1)){
      is_not_finish = false;
    }

  }

  if (angle_rotation > 180){
    angle_rotation = 360 - angle_rotation;
  }
  else{
    angle_rotation = -angle_rotation;
  }
  static int debug_cnt = 0;
  std::cout << debug_cnt++ <<" --------------------------- " << std::endl;
  std::cout << " overall index : " << (int)(360 / angle_interval1) << std::endl;
  // Output
  std::cout << " --------------------------- " << std::endl;
  std::cout << " Final Smallest Value : " << min_value << std::endl;
  std::cout << " Final Angle Interval : " << angle_rotation << std::endl;
  std::cout << " --------------------------- " << std::endl;

  pub_msg.init_angle = angle_rotation;
  pub.publish(pub_msg);


  // for(int t = 0; t <= (int)(360 / angle_interval1); t++){
  //   if(match1.angle[t] <= abs(GT_yaw) && abs(GT_yaw) <= match1.angle[t + 1]){
  //     std::cout << " Output of GT Value   : " << match1.norm12[t] << std::endl;
  //   }
  // }
  std::cout << " Compare TF Yaw Angle : " << (GT_yaw) << std::endl;


  // Output Save Index
  // std::cout << " --------------------------- " << std::endl;
  // std::cout << " Output of Save Index : " << save_index << std::endl;
  // std::cout << " Output of Save index values, angle : " << match2.angle[(save_index)] << " ranges : " << match2.ranges[(save_index)] << std::endl;

  // TF Rosbot1 to Rosbot2
  std::cout << " --------------------------- " << std::endl;
  std::cout << " Transformation Matrix Rosbot1 to Rosbot2 " << std::endl;
  std::cout << T12 << std::endl;

  // std::cout << " --- " << std::endl;
  // std::cout << " angle interval of rosbot1 scan (deg) : " << angle_interval1 << std::endl;
  // std::cout << " angle min (deg) : " << (scan1.angle_min / PI) * 180 << std::endl;
  // std::cout << " angle max (deg) : " << (scan1.angle_max / PI) * 180 << std::endl;
  // std::cout << " --- " << std::endl;
  // std::cout << " size of ranges : " << sizeof(scan1.ranges) << std::endl;
  // std::cout << " First of ranges : " << angle[0] << std::endl;
  // std::cout << " Final value of ranges : " << angle[719] << std::endl;
  // std::cout << " size of ranges " << sizeof(angle.capacity()) << std::endl;
  // std::cout << " --- " << std::endl;
  // std::cout << " Output of angle1[0] : " << angle1[0] << " Second angle1[1] : " << angle1[1] << std::endl;
  // std::cout << " Output of angle2[0] : " << angle2[0] << " Second angle2[1] : " << angle2[1] << std::endl;


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "angle_histogram");
	ros::NodeHandle nh;

	// Subscriber Initializations to all ROSBOT 1-3s

  pub = nh.advertise<rosbot_multirobot_orientation::SyncedClouds>("/rosbots/clouds_angleinitialized", 1, false);
	ros::Subscriber sub1 = nh.subscribe("/rosbots/clouds", 1, Pointcloud_callback);
	ros::spin();

	return 0;
}
