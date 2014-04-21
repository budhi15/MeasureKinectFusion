#include "addColor.h"

using namespace std;
using namespace pcl;


void addColor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, uint8_t r,uint8_t g, uint8_t b)
{
	cloud_xyzrgb->points.resize(cloud_xyz->size());
	cloud_xyzrgb->width = cloud_xyz->width;
	cloud_xyzrgb->height = cloud_xyz->height;
	for (size_t i = 0; i < cloud_xyz->points.size(); i++) {
    cloud_xyzrgb->points[i].x = cloud_xyz->points[i].x;
    cloud_xyzrgb->points[i].y = cloud_xyz->points[i].y;
	cloud_xyzrgb->points[i].z = cloud_xyz->points[i].z;
	cloud_xyzrgb->points[i].r = r;
	cloud_xyzrgb->points[i].g = g;
	cloud_xyzrgb->points[i].b = b;
}
}