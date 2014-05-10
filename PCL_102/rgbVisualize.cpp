#include "rgbVisualize.h"

#include "readPCD.h"

using namespace std;
using namespace pcl;

#define MAX_NUM_PLANES 100

void rgbVisualize(std::string planeFileName, const int numPlanes,double record[3][9])
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
	pcl::PointWithScale p_left;
    pcl::PointWithScale p_right;
	p_left.x=record[0][3];
	p_left.y=record[0][4];
	p_left.z=record[0][5];
	p_right.x=record[0][6];
	p_right.y=record[0][7];
	p_right.z=record[0][8];
	viewer->addLine (p_left, p_right, 1.0, 1.0, 1.0, "line1");
	ostringstream text1;
	text1 << record[0][2];
	pcl::PointXYZRGBA textp1;
	textp1.x=(p_left.x+p_right.x)/2;
	textp1.y=(p_left.y+p_right.y)/2;
	textp1.z=(p_left.z+p_right.z)/2;
	viewer->addText3D(text1.str(),textp1,0.1);
	p_left.x=record[1][3];
	p_left.y=record[1][4];
	p_left.z=record[1][5];
	p_right.x=record[1][6];
	p_right.y=record[1][7];
	p_right.z=record[1][8];
	viewer->addLine (p_left, p_right, 1.0, 1.0, 1.0, "line2");
	ostringstream text2;
	text2 << record[1][2];
	pcl::PointXYZRGBA textp2;
	textp2.x=(p_left.x+p_right.x)/2;
	textp2.y=(p_left.y+p_right.y)/2;
	textp2.z=(p_left.z+p_right.z)/2;
	viewer->addText3D(text2.str(),textp2,0.1);
	p_left.x=record[2][3];
	p_left.y=record[2][4];
	p_left.z=record[2][5];
	p_right.x=record[2][6];
	p_right.y=record[2][7];
	p_right.z=record[2][8];
	viewer->addLine (p_left, p_right, 1.0, 1.0, 1.0, "line3");
	ostringstream text3;
	text3 << record[2][2];
	pcl::PointXYZRGBA textp3;
	textp3.x=(p_left.x+p_right.x)/2;
	textp3.y=(p_left.y+p_right.y)/2;
	textp3.z=(p_left.z+p_right.z)/2;
	viewer->addText3D(text3.str(),textp3,0.1);
	viewer->spin();
	while (!viewer->wasStopped ())
	{
	}






}