std_msgs/Header header
bool  success

int32 idxSource
sensor_msgs/PointCloud2 cloudSource
int32 idxTarget
sensor_msgs/PointCloud2 cloudTarget

geometry_msgs/Pose initial # Source pose에서 본 Target pose
geometry_msgs/Pose output  # Source pose에서 본 Target pose
