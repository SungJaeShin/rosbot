#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "src/gicpmatcher.h"
#include <sstream>
#include <rosbot_multirobot_orientation/SyncedClouds.h>
#include <sensor_msgs/PointCloud2.h>
#include <unavlib/convt.h>

using namespace seromo;

GICPMatcher    matcher;
ros::Publisher matchingOutput;

void callbackMatching(const rosbot_multirobot_orientation::SyncedClouds::ConstPtr& msg)
{
  /**
    Note that G-ICP returns transformation matrix w.r.t. "target"!!!
    Therefore, target should be matched to previous nodes, which is train
    source <=> query
  **/
  // 1. Set Source Pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrc(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcVoxel(new pcl::PointCloud<pcl::PointXYZ>());
  *ptrSrc = unavlib::cvt::cloudmsg2cloud<pcl::PointXYZ>(msg->pc2);
//  matcher.voxelize(ptrSrc, ptrSrcVoxel, "query");

  // 2. Set Target Pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTarget(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTargetVoxel(new pcl::PointCloud<pcl::PointXYZ>());
  *ptrTarget = unavlib::cvt::cloudmsg2cloud<pcl::PointXYZ>(msg->pc1);
//  matcher.voxelize(ptrTarget, ptrTargetVoxel, "target");

  /**
   * Initial tf (initGuess) is need because ICP does not work well when the degree of rotation is large
   * tfOutput: 4x4 Transformation matrix w.r.t. target
   * tfOutput * src -> target
  **/

  // Calculate init guess
  geometry_msgs::Pose pose12 = msg->pose12;
  Eigen::Matrix4f GT_T12 = unavlib::cvt::geoPose2eigen(pose12);

  geometry_msgs::Pose initial_guess = unavlib::cvt::eigen2geoPose(GT_T12);


  // Conduct GICP
  Eigen::Matrix4f initGuessSrcToTarget = unavlib::cvt::geoPose2eigen(initial_guess);
  Eigen::Matrix4f tfOutput = matcher.gicp->calcTransformationMatrix(ptrSrc, ptrTarget, initGuessSrcToTarget);
  bool isConverged = matcher.gicp->getConvergence();

  // --------- debug --------------
  // if(isConverged)
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcTf(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ>SrcTf;
  //   pcl::transformPointCloud(*ptrSrc, *ptrSrcTf, tfOutput);
  //   SrcTf = *ptrSrcTf;

  //   DebugSrc.publish(msg->cloudSource);
  //   DebugTrg.publish(msg->cloudTarget);
  //   sensor_msgs::PointCloud2 srcTfMsg;
  //   pcl::toROSMsg(SrcTf, srcTfMsg);
  //   DebugSrcTf.publish(srcTfMsg);
  // }
  // ------------------------------

  geometry_msgs::Pose output;
  output = unavlib::cvt::eigen2geoPose(tfOutput); // .inverse()
  std::cout<<"[GICP] : "<<(int)isConverged
          <<"("<<matcher.gicp->getScore()<<")"<<std::endl;

  matchingOutput.publish(output);

  Eigen::VectorXf pred_xyzrpy = unavlib::cvt::eigen2xyzrpy(tfOutput);


  Eigen::VectorXf GT_xyzrpy = unavlib::cvt::eigen2xyzrpy(GT_T12);

  std::cout << "Predicted | T 1->2 " << std::endl;
  std::cout<< output << std::endl;
  std::cout << "GT | T 1->2 " << std::endl;
  std::cout<< GT_T12 << std::endl;
  std::cout << " " << std::endl;
  std::cout << " " << std::endl;
  std::cout << "Predicted | Yaw 1->2 (deg): " << pred_xyzrpy(5)/3.14*180 << std::endl;
  std::cout << "GT | Yaw 1->2 (deg): " << GT_xyzrpy(5)/3.14*180 << std::endl;
  std::cout << "GT | Distance: " << pow(pow(GT_xyzrpy(0), 2) + pow(GT_xyzrpy(1), 2), 0.5) << std::endl;
  std::cout << "__________________" << std::endl;
  std::cout << " " << std::endl;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbot_gicp");

    ros::NodeHandle nodeHandler;

    float maxDist, scoreLimit, voxelSizeQuery, voxelSizeTarget, epsilon;
    int numMaxIter, numIter;
    nodeHandler.param<float>("/ICP/maxdist", maxDist, 21.0);
    nodeHandler.param<float>("/ICP/minscore", scoreLimit, 0.25);
    nodeHandler.param<float>("/ICP/voxelsizeQuery", voxelSizeQuery, 0.3);
    nodeHandler.param<float>("/ICP/voxelsizeTarget", voxelSizeTarget, 0.3);
    nodeHandler.param("/ICP/maxiter", numMaxIter, 1100);
    nodeHandler.param("/ICP/RANSACIter", numIter, 550);
    nodeHandler.param<float>("/ICP/epsilon", epsilon, 0.012);

    matcher.setValue(maxDist, scoreLimit, numMaxIter, numIter, voxelSizeQuery, voxelSizeTarget, epsilon);

    matchingOutput = nodeHandler.advertise<geometry_msgs::Pose>("/rosbot12/gicp/output", 100);

    static ros::Subscriber subMatchingPair =
        nodeHandler.subscribe<rosbot_multirobot_orientation::SyncedClouds>("/rosbots/clouds", 1 , &callbackMatching);

    ros::spin();

    return 0;
}
