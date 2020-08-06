#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <mutex>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <rosbot_multirobot_orientation/SyncedClouds.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


class SyncFilter {
	public:
		SyncFilter();
		void Laserscan1_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg);
		void Laserscan2_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg);
		void Laserscan3_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg);

	private:
		ros::NodeHandle nh;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener tfListener_;

		//Publishers and subscribers
		ros::Publisher pub;
		ros::Publisher pub1_debug;
		ros::Publisher pub2_debug;
		ros::Publisher pub3_debug;
		ros::Subscriber sub1;
		ros::Subscriber sub2;
		ros::Subscriber sub3;

		//Messages
		rosbot_multirobot_orientation::SyncedClouds pub_msg;
		rosbot_multirobot_orientation::SyncedClouds old_pub_msg;
		sensor_msgs::PointCloud2 pub_msg_1cloud;
		sensor_msgs::PointCloud2 pub_msg_2cloud;
		sensor_msgs::PointCloud2 pub_msg_3cloud;
};

SyncFilter::SyncFilter(){
	// Assign Publisher object with nodehandle
	pub = nh.advertise<rosbot_multirobot_orientation::SyncedClouds>("/rosbots/clouds", 100, false);
	pub1_debug = nh.advertise<sensor_msgs::PointCloud2>("/rosbot1/cloud", 100, false);
	pub2_debug = nh.advertise<sensor_msgs::PointCloud2>("/rosbot2/cloud", 100, false);
	pub3_debug = nh.advertise<sensor_msgs::PointCloud2>("/rosbot3/cloud", 100, false);

	// Subscriber Initializations to all ROSBOT 1-3s
	sub1 = nh.subscribe("/rosbot1/scan", 100, &SyncFilter::Laserscan1_callback, this);
	sub2 = nh.subscribe("/rosbot2/scan", 100, &SyncFilter::Laserscan2_callback, this);
	sub3 = nh.subscribe("/rosbot3/scan", 100, &SyncFilter::Laserscan3_callback, this);

}

void SyncFilter::Laserscan1_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg)
{
	std::mutex mtx;


	sensor_msgs::PointCloud2 cloud;

	projector_.transformLaserScanToPointCloud("rosbot1_tf/laser", *sub_msg, cloud, tfListener_);
	mtx.lock();
	pub_msg.pc1 = cloud;
	mtx.unlock();
    pub_msg_1cloud = cloud;
    pub1_debug.publish(pub_msg_1cloud);
}

void SyncFilter::Laserscan2_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg)
{
	sensor_msgs::PointCloud2 cloud;
	laser_geometry::LaserProjection projector_;
	projector_.transformLaserScanToPointCloud("rosbot2_tf/laser", *sub_msg, cloud, tfListener_);

	std::mutex mtx;
    mtx.lock();
    
    pub_msg.pc2 = cloud;

    mtx.unlock();
    pub_msg_2cloud = cloud;
    pub2_debug.publish(pub_msg_2cloud);

}

void SyncFilter::Laserscan3_callback(const sensor_msgs::LaserScan::ConstPtr& sub_msg)
{
	sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScanToPointCloud("rosbot3_tf/laser", *sub_msg, cloud, tfListener_);

	std::mutex mtx;
    mtx.lock();
    
    pub_msg.pc3 = cloud;
    
    mtx.unlock();
    pub_msg_3cloud = cloud;
    pub1_debug.publish(pub_msg_3cloud);

    bool is_pc1_updated = pub_msg.pc1.header.stamp != old_pub_msg.pc1.header.stamp;
    bool is_pc2_updated = pub_msg.pc2.header.stamp != old_pub_msg.pc2.header.stamp;

    if(is_pc1_updated && is_pc2_updated){
    	pub_msg.header.stamp = ros::Time::now();
    	pub.publish(pub_msg);
    	old_pub_msg = pub_msg;
    }
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "sync_laserscans");
	
	SyncFilter syncfilter;
	
	ros::spin();

	return 0;
}