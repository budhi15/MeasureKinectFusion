// PCL_101.cpp : Defines the entry point for the console application.
//

#include "commonheaders.h"
#include "readPCD.h"
#include "convertOBJtoPCD.h"
#include "filterPCD.h"
//#include "segmentPlanes.h"
#include "visualizePCD.h"
#include "visualizePlanes.h"


using namespace std;
using namespace pcl;
int main (int argc, char** argv)
{
	if(argc < 2)
	{
		cerr<<"Incorrect use"<<endl;
		Sleep(2000);
		exit(0);
	}
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
	PointCloud<PointXYZ>::Ptr filteredCloud;
	string pointCloudName = argv[1];
	string PCDfileName = pointCloudName + "_filtered"+".pcd";
	ifstream tempHandle;
	tempHandle.open(PCDfileName);
	if(!tempHandle.is_open())
	{
		cout<<"Filtered PCD file doesn't exist. Looking for unfiltered file."<<endl;
		PCDfileName = pointCloudName + ".pcd";
		tempHandle.open(PCDfileName);
		if(!tempHandle.is_open())
		{
			cout<<"PCD file doesn't exist. Converting from OBJ file."<<endl;
			string OBJfileName = pointCloudName + ".obj";
			PCDfileName = convertOBJtoPCD(OBJfileName);
		}
		readPCD(PCDfileName, cloud);
		filteredCloud = filterPCD(pointCloudName+"_filtered.pcd",cloud); 
	}
	else
	{
		//the input file is already filtered
		readPCD(PCDfileName, cloud);
		PointCloud<PointXYZ>::Ptr tempPtr (new PointCloud<PointXYZ>);
		filteredCloud = tempPtr;
		fromROSMsg(*cloud, *filteredCloud);
	}

	//int numPlanes = segmentPlanes(pointCloudName, filteredCloud);

	PCDWriter writer;
	PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ>), cloud_f (new PointCloud<PointXYZ>);
	ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	PointIndices::Ptr inliers (new PointIndices ());
	// Create the segmentation object
	SACSegmentation<PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	ExtractIndices<PointXYZ> extract;

	int i = 0, nr_points = (int) filteredCloud->points.size ();
	cout<<"Plane Segmentation started"<<endl;
	// While 30% of the original cloud is still there
	vector<ModelCoefficients> allcoefficients;
	while (filteredCloud->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (filteredCloud);
		seg.segment (*inliers, *coefficients);
		allcoefficients.push_back(*coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (filteredCloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud "<<i<<" representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		stringstream ss;
		ss << pointCloudName << "_plane_"<<i << ".pcd";
		stringstream sss;
		sss << pointCloudName << "_plane_"<<i << ".txt";
		ofstream writeparam(sss.str ());
		writeparam << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << endl;
		writeparam.close();

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		filteredCloud.swap (cloud_f);
		i++;
	}
	stringstream s1;
	s1 << pointCloudName << "_parallel.txt";
	ofstream writeparallel(s1.str ());
	stringstream s2;
	s2 << pointCloudName << "_perp.txt";
	ofstream writeperp(s2.str ());
	for (int i=0;i<allcoefficients.size();i++)
	{
		for (int j=0;j<allcoefficients.size();j++)
		{
			if (i!=j)
			{
				if (abs(allcoefficients[i].values[0]*allcoefficients[j].values[0]+allcoefficients[i].values[1]*allcoefficients[j].values[1]+allcoefficients[i].values[2]*allcoefficients[j].values[2])<=0.1)
				{
					writeperp << "pair: " << i << " " << j << endl;
				}
				if (abs(allcoefficients[i].values[0]*allcoefficients[j].values[0]+allcoefficients[i].values[1]*allcoefficients[j].values[1]+allcoefficients[i].values[2]*allcoefficients[j].values[2])>=0.9)
				{
					writeparallel << "pair: " << i << " " << j << endl;
				}
			}
		}
	}
	writeparallel.close();
	writeperp.close();
	cout<<"Plane Segmentation complete"<<endl; 

	//visualizePCD(filteredCloud);
	//visualizePlanes(pointCloudName, numPlanes);

	getchar();
	return (0);
}


