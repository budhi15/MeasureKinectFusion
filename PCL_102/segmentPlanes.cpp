#include "segmentPlanes.h"
using namespace std;
using namespace pcl;

int segmentPlane(string pointCloudName,PointCloud<PointXYZ>::Ptr filteredCloud)
{
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
		while (filteredCloud->points.size () > 0.3 * nr_points)
		{
		i++;
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (filteredCloud);
		seg.segment (*inliers, *coefficients);
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
		writer.write<PointXYZ> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		filteredCloud.swap (cloud_f);
		
		}
	  
		cout<<"Plane Segmentation complete"<<endl;
		return i;
}