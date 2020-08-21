#ifndef GICP_H
#define GICP_H
#define ICP_RESULT_OUT
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>

struct GICPOutput
{
  Eigen::Matrix4f transformation;
  float score;
  bool converged;
};

class GICP
{
public:
  GICP(float maxDist, int numMaxIter, float epsilon, int numIter, float scoreUpperBoundary);
  ~GICP();
  GICPOutput iterate(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr target);
  Eigen::Matrix4f calcTransformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f tf_src2target);
  float getScore();
  bool getConvergence();

private:
  bool first_in;
  bool isConverged;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
  float fitnessScore;
  float scoreLimit;
};

#endif // GICP_CLASS_H
