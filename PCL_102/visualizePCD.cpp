#include "visualizePCD.h"
using namespace pcl;

void visualizePCD(PointCloud<PointXYZ>::Ptr cloud)
{
visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
}

void visualizePCD(PointCloud<PointXYZRGB>::Ptr cloud)
{
visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
}