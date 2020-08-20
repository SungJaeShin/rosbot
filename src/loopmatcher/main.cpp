#include <ros/ros.h>
#include <std_msgs/String.h>
#include "gicpmatcher.h"
#include <sstream>
#include <rgtSLAM/matcher.h>
#include "common.h"
#include <sensor_msgs/PointCloud2.h>

using namespace seromo;

GICPMatcher    matcher;
ros::Publisher matchingOutput;
ros::Publisher DebugSrc;
ros::Publisher DebugSrcTf;
ros::Publisher DebugTrg;

void callbackMatching(const rgtSLAM::matcher::ConstPtr& msg)
{
  /**
    Note that G-ICP returns transformation matrix w.r.t. "target"!!!
    Therefore, target should be matched to previous nodes, which is train
    source <=> query
  **/
  // 1. Set Source Pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrc(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcVoxel(new pcl::PointCloud<pcl::PointXYZ>());
  *ptrSrc = rgtlib::msgToPcl<pcl::PointXYZ>(msg->cloudSource);
//  matcher.voxelize(ptrSrc, ptrSrcVoxel, "query");

  // 2. Set Target Pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTarget(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTargetVoxel(new pcl::PointCloud<pcl::PointXYZ>());
  *ptrTarget = rgtlib::msgToPcl<pcl::PointXYZ>(msg->cloudTarget);
//  matcher.voxelize(ptrTarget, ptrTargetVoxel, "target");

  /**
   * Initial tf (initGuess) is need because ICP does not work well when the degree of rotation is large
   * tfOutput: 4x4 Transformation matrix w.r.t. target
   * tfOutput * src -> target
  **/
  Eigen::Matrix4f initGuessSrcToTarget = rgtlib::poseToEigen(msg->initial);
  Eigen::Matrix4f tfOutput = matcher.gicp->calcTransformationMatrix(ptrSrc, ptrTarget, initGuessSrcToTarget);
  bool isConverged = matcher.gicp->getConvergence();

  // --------- debug --------------
  if(isConverged)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcTf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>SrcTf;
    pcl::transformPointCloud(*ptrSrc, *ptrSrcTf, tfOutput);
    SrcTf = *ptrSrcTf;

    DebugSrc.publish(msg->cloudSource);
    DebugTrg.publish(msg->cloudTarget);
    sensor_msgs::PointCloud2 srcTfMsg = rgtlib::pclToMsg(SrcTf);
    DebugSrcTf.publish(srcTfMsg);
  }
  // ------------------------------

  rgtSLAM::matcher output = *msg;
  output.success = isConverged;
  output.output = rgtlib::eigenToPose(tfOutput.inverse());
  std::cout<<"[LOOPMATCHER] ICP OF "<<msg->idxSource<<","<<msg->idxTarget<<" : "<<(int)output.success
          <<"("<<matcher.gicp->getScore()<<")"<<std::endl;

  matchingOutput.publish(output);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gicpmatcher");

    ros::NodeHandle nodeHandler;

    float maxDist, scoreLimit, voxelSizeQuery, voxelSizeTarget, epsilon;
    int numMaxIter, numIter;
    nodeHandler.param<float>("/ICP/maxdist", maxDist, 21.0);
    nodeHandler.param<float>("/ICP/minscore", scoreLimit, 0.25);
    nodeHandler.param<float>("/ICP/voxelsizeQuery", voxelSizeQuery, 0.2);
    nodeHandler.param<float>("/ICP/voxelsizeTarget", voxelSizeTarget, 0.4);
    nodeHandler.param("/ICP/maxiter", numMaxIter, 1100);
    nodeHandler.param("/ICP/RANSACIter", numIter, 550);
    nodeHandler.param<float>("/ICP/epsilon", epsilon, 0.012);

    matcher.setValue(maxDist, scoreLimit, numMaxIter, numIter, voxelSizeQuery, voxelSizeTarget, epsilon);

    matchingOutput = nodeHandler.advertise<rgtSLAM::matcher>("/loopmatcher/matchingOutput", 100);
    DebugSrc = nodeHandler.advertise<sensor_msgs::PointCloud2>("/loopmatcher/debug/source", 100);
    DebugTrg = nodeHandler.advertise<sensor_msgs::PointCloud2>("/loopmatcher/debug/target", 100);
    DebugSrcTf = nodeHandler.advertise<sensor_msgs::PointCloud2>("/loopmatcher/debug/source_tf", 100);

    static ros::Subscriber subMatchingPair =
        nodeHandler.subscribe<rgtSLAM::matcher>("/loopmatcher/candidate", 1 , &callbackMatching);

    ros::spin();

    return 0;
}
