#include "rgbVisualize.h"

#include "readPCD.h"

using namespace std;
using namespace pcl;

#define MAX_NUM_PLANES 10

void rgbVisualize(std::string planeFileName, const int numPlanes)
{
	string fileName;
	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		
	 PointCloud<PointXYZRGB>::Ptr planeCloudXYZRGB[MAX_NUM_PLANES];
	 
	 viewer->setBackgroundColor (0, 0, 0);
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem ();
  viewer->initCameraParameters ();
	for (int i=1; i<=numPlanes; i++)
	{
		ostringstream convert;
		convert << i;
		fileName = planeFileName + "_plane_"+ convert.str()+".pcd";
		sensor_msgs::PointCloud2::Ptr planeCloud (new sensor_msgs::PointCloud2);
		readPCD(fileName, planeCloud) ;
		PointCloud<PointXYZRGB>::Ptr tempPtr (new PointCloud<PointXYZRGB>);
		planeCloudXYZRGB[i-1] = tempPtr;
		fromROSMsg(*planeCloud, *planeCloudXYZRGB[i-1]);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(planeCloudXYZRGB[i-1]);
		viewer->addPointCloud<pcl::PointXYZRGB> (planeCloudXYZRGB[i-1], rgb, "sample cloud"+convert.str());
	}
	viewer->spin();
	 while (!viewer->wasStopped ())
   {
   }






}