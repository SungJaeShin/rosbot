/** @file gicpmatcher.h
    @date 2020/08
    @author Hyungtae Lim
*/

#ifndef GICPMATCHER_H
#define GICPMATCHER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unistd.h>
#include <geometry_msgs/Pose.h>

#include "gicp.h"
#include <iostream>
#include "rosbot_gicp/gicpMsg.h"
//#include <common.h>


namespace seromo
{

    class GICPMatcher
    {
    private:
        float scoreLimit; /**< Maximum score of acceptance */
        float maxDist; /**< Maximum distance between two nodes of acceptance */
        int numMaxIter; /**< Max iteration count of g-icp */
        int numIter; /**< RANSAC iteration value of g-icp */
        float epsilon; /**< Epsilon value of g-icp */
        float voxelSizeQuery;
        float voxelSizeTarget;
//        /** For the best fit node among train */
//        float min_score;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_best_pc;
//        Eigen::Matrix4f best_transform;
//        int best_idx;

        pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;

    public:
        GICP * gicp; /**< GICP class (in gicp_class folder) */

        GICPMatcher();
        ~GICPMatcher();
        void setValue(float &maxCorrespondenseDist, float &scoreUpperBoundary, int& numRANSACMaxIter, int& numRANSACIter, float& VoxelizationQuery, float& VoxelizationTarget, float& matchingEpsilon);
        void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst, std::string mode);
    };

}


#endif

