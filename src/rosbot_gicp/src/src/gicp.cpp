#include "gicp.h"

GICP::GICP(float maxDist, int numMaxIter, float epsilon, int numIter, float scoreUpperBoundary)
{
  gicp.setTransformationEpsilon(epsilon);
  gicp.setRANSACIterations(numIter);
  gicp.setMaxCorrespondenceDistance(maxDist);
  gicp.setMaximumIterations(numMaxIter);
  scoreLimit = scoreUpperBoundary;
  first_in = true;
}

GICP::~GICP()
{

}

GICPOutput GICP::iterate(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
  GICPOutput output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr dummy(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.setInputSource(src);
  gicp.setInputTarget(target);
  gicp.align(*dummy);
  output.transformation = gicp.getFinalTransformation();
  output.score = gicp.getFitnessScore();
  output.converged = gicp.hasConverged();
  return output;
}

Eigen::Matrix4f GICP::calcTransformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f tf_src2target)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr srcInitGuess(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*src, *srcInitGuess, tf_src2target);

  GICPOutput matchingOutput = iterate(srcInitGuess, target);
  Eigen::Matrix4f tfMat;
  fitnessScore = matchingOutput.score;
  isConverged = matchingOutput.converged;
  if((matchingOutput.score > scoreLimit) || !(matchingOutput.converged)){
    tfMat = tf_src2target;
  }
  else{
    tfMat = matchingOutput.transformation * tf_src2target;
  }
  return tfMat;
}

float GICP::getScore()
{
  return fitnessScore;
}

bool GICP::getConvergence()
{
  return fitnessScore<scoreLimit;
}
