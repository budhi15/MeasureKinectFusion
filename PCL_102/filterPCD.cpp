#include "filterPCD.h"

using namespace std;
using namespace pcl;

PointCloud<PointXYZ>::Ptr filterPCD(string filteredCloudFileName, sensor_msgs::PointCloud2::Ptr inputCloud)
{
	PointCloud<PointXYZ>::Ptr filteredCloud (new PointCloud<PointXYZ>);
	sensor_msgs::PointCloud2::Ptr filtCloud (new sensor_msgs::PointCloud2);
		 
  VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (inputCloud);
<<<<<<< HEAD
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
=======
  sor.setLeafSize (0.07f, 0.07f, 0.07f);
>>>>>>> 793034ab95b91b810581d35dbfdadd923411cd16
  sor.filter (*filtCloud);

  std::cerr << "PointCloud after filtering: " << filtCloud->width * filtCloud->height 
       << " data points (" << pcl::getFieldsList (*filtCloud) << ").";
	fromROSMsg(*filtCloud, *filteredCloud);
	PCDWriter writer;
    writer.write<pcl::PointXYZ> (filteredCloudFileName, *filteredCloud, false);
	return filteredCloud;
}
