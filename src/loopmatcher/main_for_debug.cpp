#include <ros/ros.h>
#include <std_msgs/String.h>
#include "gicpmatcher.h"
#include <sstream>
#include <rgtSLAM/matcher.h>
#include "common.h"
#include <random>
using namespace seromo;

GICPMatcher    matcher;
ros::Publisher matchingOutput;


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

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ point_xyz; // pcl::PointXYZ이라는 type에 data를 담는다.
    for (int k=0; k < 100 ; ++k){
      float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
      float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
      float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
      point_xyz.x = x;     point_xyz.y = y;     point_xyz.z = z;
      cloud.push_back(point_xyz);
    }

    pcl::PointCloud<pcl::PointXYZ> target;

    for (auto const &pt : cloud.points){
        pcl::PointXYZ point_xyz;
        point_xyz.x = pt.x + 0.70f;
        point_xyz.y = pt.y + 0.5f;
        point_xyz.z = pt.z - 0.1f;
        target.points.push_back(point_xyz);
    }

    // 1. Set Source Pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcVoxel(new pcl::PointCloud<pcl::PointXYZ>());
    *ptrSrc = cloud;
    matcher.voxelize(ptrSrc, ptrSrcVoxel, "query");

    // 2. Set Target Pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTarget(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTargetVoxel(new pcl::PointCloud<pcl::PointXYZ>());
    *ptrTarget = target;
    matcher.voxelize(ptrTarget, ptrTargetVoxel, "target");

    /**
     * Initial tf (initGuess) is need because ICP does not work well when the degree of rotation is large
     * tfOutput: 4x4 Transformation matrix w.r.t. target
    **/
    Eigen::Matrix4f initial;

    initial<< 1,            0,       0,  0.70,
              0,            1,       0,  0.2,
              0,            0,       1,  0.0271,
              0,            0,       0,    1;
    Eigen::Matrix4f initGuessSrcToTarget = initial;
    Eigen::Matrix4f tfOutput = matcher.gicp->calcTransformationMatrix(ptrSrcVoxel, ptrTargetVoxel, initGuessSrcToTarget);
    std::cout<<"Output: "<<std::endl;
    std::cout<<tfOutput<<std::endl;
    bool isConverged = matcher.gicp->getConvergence();


    ros::spin();

    return 0;
}
