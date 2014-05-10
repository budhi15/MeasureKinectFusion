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
	string pointCloudName = argv[1];
	int numplanes;

	if (false)
	{
		sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
		PointCloud<PointXYZ>::Ptr mergedCloud;
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

		uint8_t color[90][3] = {255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
			0,255,0, //green
			0,0,255, //blue
			255,255,0, //
			0,255,255, //
			255,0,255, //
			128,128,0, //
			0,128,128, //
			128,0,128, //
			128,128,128, //
			255,0,0, //red
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
		vector<PointXYZ> planecenter;
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
		while (mergedCloud->points.size () > 0.5 * nr_points)
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

			PointXYZ pcenter(0,0,0);
			for (size_t i = 0; i < cloud_p->points.size(); i++) {
				pcenter.x += cloud_p->points[i].x;
				pcenter.y += cloud_p->points[i].y;
				pcenter.z += cloud_p->points[i].z;
			}
			pcenter.x /= cloud_p->points.size();
			pcenter.y /= cloud_p->points.size();
			pcenter.z /= cloud_p->points.size();
			planecenter.push_back(pcenter);

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
		double record[3][9];
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<9;j++)
			{
				record[i][j]=0;
			}
		}
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
						double tdist = (allcoefficients[i].values[0]*planecenter[j].x+allcoefficients[i].values[1]*planecenter[j].y+allcoefficients[i].values[2]*planecenter[j].z+allcoefficients[i].values[3])/sqrt(pow(allcoefficients[i].values[0],2)+pow(allcoefficients[i].values[1],2)+pow(allcoefficients[i].values[2],2));
						double dist=abs(tdist);
						double tx = planecenter[j].x - allcoefficients[i].values[0]*tdist;
						double ty = planecenter[j].y - allcoefficients[i].values[1]*tdist;
						double tz = planecenter[j].z - allcoefficients[i].values[2]*tdist;
						if (abs(allcoefficients[i].values[0])>0.9 && dist>record[0][2])
						{
							record[0][2]=dist;
							record[0][0]=i;
							record[0][1]=j;
							record[0][3]=planecenter[j].x;
							record[0][4]=planecenter[j].y;
							record[0][5]=planecenter[j].z;
							record[0][6]=tx;
							record[0][7]=ty;
							record[0][8]=tz;
						}
						if (abs(allcoefficients[i].values[1])>0.9 && dist>record[1][2])
						{
							record[1][2]=dist;
							record[1][0]=i;
							record[1][1]=j;
							record[1][3]=planecenter[j].x;
							record[1][4]=planecenter[j].y;
							record[1][5]=planecenter[j].z;
							record[1][6]=tx;
							record[1][7]=ty;
							record[1][8]=tz;
						}
						if (abs(allcoefficients[i].values[2])>0.9 && dist>record[2][2])
						{
							record[2][2]=dist;
							record[2][0]=i;
							record[2][1]=j;
							record[2][3]=planecenter[j].x;
							record[2][4]=planecenter[j].y;
							record[2][5]=planecenter[j].z;
							record[2][6]=tx;
							record[2][7]=ty;
							record[2][8]=tz;
						}
						writeparallel << "pair: " << i << " " << j << " " << dist << endl;
						writeparallel << allcoefficients[i].values[0] << " " << allcoefficients[i].values[1] << " " << allcoefficients[i].values[2] << " " << allcoefficients[i].values[3] << endl;
						writeparallel << allcoefficients[j].values[0] << " " << allcoefficients[j].values[1] << " " << allcoefficients[j].values[2] << " " << allcoefficients[j].values[3] << endl;
					}
				}
			}
		}
		writeparallel.close();
		writeperp.close();
		stringstream s3;
		s3 << folder << pointCloudName << "_record.txt";
		ofstream writerecord(s3.str ());
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<9;j++)
			{
				writerecord << record[i][j] << " ";
			}
			writerecord << endl;
		}
		writerecord.close();
		cout<<"Plane Segmentation complete"<<endl;
		visualizePCD(mergedCloud);
		//visualizePlanes(pointCloudName, i);
		numplanes=allcoefficients.size();
	}
	numplanes=87;
	stringstream s4;
	s4 << folder << pointCloudName << "_record.txt";
	ifstream readrecord(s4.str ());
	double recordr[3][9];
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<9;j++)
		{
			readrecord >> recordr[i][j];
		}
	}
	readrecord.close();
	rgbVisualize(folder+pointCloudName, numplanes,recordr);
	getchar();
	return (0);
}
