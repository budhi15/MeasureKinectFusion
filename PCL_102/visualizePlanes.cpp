#include "visualizePlanes.h"
#include "readPCD.h"
#include "visualizePCD.h"

using namespace std;
using namespace pcl;

void visualizePlanes(std::string planeFileName, int numPlanes)
{
	string fileName;
	
	for (int i=1; i<=numPlanes; i++)
	{
		ostringstream convert;
		convert << i;
		fileName = planeFileName + "_plane_"+ convert.str()+".pcd";
		sensor_msgs::PointCloud2::Ptr planeCloud (new sensor_msgs::PointCloud2);
		readPCD(fileName, planeCloud) ;
		PointCloud<PointXYZ>::Ptr planeCloudXYZ (new PointCloud<PointXYZ>);
		fromROSMsg(*planeCloud, *planeCloudXYZ);
		visualizePCD(planeCloudXYZ);		
	}








}