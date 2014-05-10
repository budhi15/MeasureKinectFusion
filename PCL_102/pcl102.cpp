// PCL_101.cpp : Defines the entry point for the console application.
//

#include "commonheaders.h"
#include "readPCD.h"
#include "convertOBJtoPCD.h"
#include "filterPCD.h"
//#include "segmentPlanes.h"
#include "visualizePCD.h"
#include "visualizePlanes.h"
#include "addColor.h"
#include "rgbVisualize.h"
#include "pairAlign.h"
#include "rigidTransform.h"
#define MAX_CLOUDS 10

using namespace std;
using namespace pcl;


struct listOfFiles
{
	string allFiles[MAX_CLOUDS];
	int count;
};

listOfFiles getListOfInputFiles(string fileWithNames)
{
	ifstream tempHandle;
	tempHandle.open(fileWithNames);
	listOfFiles returnStuff;
	string multipleFiles[MAX_CLOUDS];

	if(!tempHandle.is_open())
	{
		cerr<<"Could not open the input file list."<<endl;
		
		Sleep(2000);
		exit(0);
			
	}
	else
	{
		
		string line;
		int readCounter = 0;
		while(getline(tempHandle, line))
		{
			returnStuff.allFiles[readCounter] = line;
			readCounter++;
		}

		
		returnStuff.count = readCounter;

	}
	return returnStuff;

}

int main (int argc, char** argv)
{
	if(argc < 2)
	{
		cerr<<"Incorrect use"<<endl;
		Sleep(2000);
		exit(0);
	}
	string folder = "data\\";
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
	PointCloud<PointXYZ>::Ptr mergedCloud;
	string pointCloudName = argv[1];
	string PCDfileName = folder + pointCloudName + "_merged"+".pcd";
	ifstream tempHandle;
	tempHandle.open(PCDfileName);
	bool isMerged = false;

	if(!tempHandle.is_open())
	{
		//the merged pcd doesn't exist. Will need to merge all the individual point clouds
		
		string textFile = folder + pointCloudName + ".txt";
		listOfFiles allClouds = getListOfInputFiles(textFile);
		PointCloud<PointXYZ>::Ptr filteredCloud[MAX_CLOUDS];
		for(int i=0; i<allClouds.count; i++)
		{
			//read individual files

			sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
			
			
			string PCDfileName = folder + allClouds.allFiles[i] + "_filtered"+".pcd";
			ifstream tempHandle;
			tempHandle.open(PCDfileName);
			if(!tempHandle.is_open())
			{
				cout<<"Filtered PCD file doesn't exist. Looking for unfiltered file."<<endl;
				PCDfileName = folder + allClouds.allFiles[i] + ".pcd";
				tempHandle.open(PCDfileName);
				if(!tempHandle.is_open())
				{
					cout<<"PCD file doesn't exist. Converting from OBJ file."<<endl;
					string OBJfileName = folder + allClouds.allFiles[i] + ".obj";
					PCDfileName = convertOBJtoPCD(OBJfileName);
				}
				readPCD(PCDfileName, cloud);
				filteredCloud[i] = filterPCD(folder+allClouds.allFiles[i]+"_filtered.pcd",cloud); 
			}
			else
			{
				//the input file is already filtered
				readPCD(PCDfileName, cloud);
				PointCloud<PointXYZ>::Ptr tempPtr (new PointCloud<PointXYZ>);
				filteredCloud[i] = tempPtr;
				fromROSMsg(*cloud, *filteredCloud[i]);
			}

		}

		//merge all the individual point clouds using LOT OF THINGS!
		Eigen::Matrix4f transMat;
		//rigidTransform(filteredCloud[1] , filteredCloud[0], transMat, folder+PCDfileName);
		string temp11 = folder+pointCloudName;
		correspondences_demo(filteredCloud[0], filteredCloud[1], temp11.c_str(), transMat);
		PointCloud<PointXYZ>::Ptr temper (new PointCloud<PointXYZ>);
		transformPointCloud (*filteredCloud[1], *temper, transMat);
		 pcl::visualization::PCLVisualizer viz;
		viz.addPointCloud (filteredCloud[1], "original");
		viz.addPointCloud (temper, "rotated");
		viz.spin ();

		pcl::visualization::PCLVisualizer viz1;
		viz1.addPointCloud (filteredCloud[0], "cloud0");
		viz1.addPointCloud (temper, "cloud1");
		viz1.spin ();
		PointCloud<PointXYZ>::Ptr temp (new PointCloud<PointXYZ>), tempPtr (new PointCloud<PointXYZ>);
		Eigen::Matrix4f transform;
		pairAlign (filteredCloud[0], filteredCloud[1], temp, transform, true);
		pcl::transformPointCloud (*temp, *tempPtr, transform);
		mergedCloud = tempPtr;
		std::stringstream ss;
		ss << folder << pointCloudName << "_merged.pcd";
		//io::savePCDFileASCII (ss.str (), *mergedCloud);
	}

	else
	{
		isMerged = true;
		readPCD(PCDfileName, cloud);
		PointCloud<PointXYZ>::Ptr tempPtr (new PointCloud<PointXYZ>);
		mergedCloud = tempPtr;
		fromROSMsg(*cloud, *mergedCloud);
	}



	


	//int numPlanes = segmentPlanes(pointCloudName, filteredCloud);
	visualizePCD(mergedCloud);
	
	uint8_t color[10][3] = {255,0,0, //red
						0,255,0, //green
						0,0,255, //blue
						255,255,0, //
						0,255,255, //
						255,0,255, //
						128,128,0, //
						0,128,128, //
						128,0,128, //
						128,128,128, //
	};
	PCDWriter writer;
	PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ>), cloud_f (new PointCloud<PointXYZ>);
	PointCloud<PointXYZRGB>:: Ptr cloud_p_color(new PointCloud<PointXYZRGB>);
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

	int i = 0, nr_points = (int) mergedCloud->points.size ();
	cout<<"Plane Segmentation started"<<endl;
	// While 30% of the original cloud is still there
	vector<ModelCoefficients> allcoefficients;
	while (mergedCloud->points.size () > 0.3 * nr_points)
	{
		i++;
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (mergedCloud);
		seg.segment (*inliers, *coefficients);
		allcoefficients.push_back(*coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (mergedCloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud "<<i<<" representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		
		
		addColor(cloud_p, cloud_p_color, color[i-1][0], color[i-1][1], color[i-1][2]);
		stringstream ss;
		ss << folder << pointCloudName << "_plane_"<<i << ".pcd";
		writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p_color, false);
		stringstream sss;
		sss << folder << pointCloudName << "_plane_"<<i << ".txt";
		ofstream writeparam(sss.str ());
		writeparam << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << endl;
		writeparam.close();

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		mergedCloud.swap (cloud_f);
		
	}
	stringstream s1;
	s1 << folder << pointCloudName << "_parallel.txt";
	ofstream writeparallel(s1.str ());
	stringstream s2;
	s2 << folder << pointCloudName << "_perp.txt";
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

	visualizePCD(mergedCloud);
	//visualizePlanes(pointCloudName, i);
	rgbVisualize(pointCloudName, i);
	getchar();
	return (0);
}
