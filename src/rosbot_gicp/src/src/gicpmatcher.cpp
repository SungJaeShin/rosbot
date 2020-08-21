#include "gicpmatcher.h"

using namespace seromo;


GICPMatcher::GICPMatcher()
{
}


void GICPMatcher::setValue(float &maxCorrespondenseDist, float &scoreUpperBoundary, int& numRANSACMaxIter, int& numRANSACIter, float& VoxelizationQuery, float& VoxelizationTarget, float& matchingEpsilon){
  maxDist = maxCorrespondenseDist;
  scoreLimit = scoreUpperBoundary;
  voxelSizeQuery = VoxelizationQuery;
  voxelSizeTarget = VoxelizationTarget;
  numMaxIter = numRANSACMaxIter;
  numIter = numRANSACIter;
  epsilon = matchingEpsilon;
  // Initialization

  gicp = new GICP(maxDist, numMaxIter, epsilon, numIter, scoreLimit);
}


GICPMatcher::~GICPMatcher()
{
}

void GICPMatcher::voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst, std::string mode){
  if (mode == "query"){
    voxelfilter.setLeafSize(voxelSizeQuery, voxelSizeQuery, voxelSizeQuery);
  }else if (mode == "target") {
    voxelfilter.setLeafSize(voxelSizeTarget, voxelSizeTarget, voxelSizeTarget);
}else{
  std::cout<<"[GICP Matcher]: Wrong mode is comming!"<<std::endl;
  voxelfilter.setLeafSize(voxelSizeQuery, voxelSizeQuery, voxelSizeQuery);
}


  voxelfilter.setInputCloud(src);
  voxelfilter.filter(*dst);
}


