#include "commonheaders.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr filterPCD(std::string filteredCloudFileName, sensor_msgs::PointCloud2::Ptr inputCloud);