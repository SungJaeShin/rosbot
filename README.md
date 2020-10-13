# rosbot_description #

URDF model for Gazebo integrated with ROS.

## How to use. ## 

* Launch the virtual environment with multiple robots

  *$ roslaunch rosbot_gazebo main.lanch*

* Synchronize Laserscans

  *$ rosrun rosbot_multirobot_orientation sync_laserscans*

* Conduct yaw initialization with angle histogram matching

  *$ rosrun rosbot_multirobot_orientation angle_histogram*

* Conduct pointcloud registration algorithms for sophisticated estimation of relative pose

  **NDT**: *$ rosrun rosbot_multirobot_orientation rosbot_ndt*
  
  **ICP**: *$ rosrun rosbot_multirobot_orientation rosbot_icp*
  
  **G-ICP**: *$ rosrun rosbot_gicp rosbot_gicp*
