#include "readPCD.h"

using namespace std;
using namespace pcl;


void readPCD(string PCDfileName, sensor_msgs::PointCloud2::Ptr cloud)
{
	cout << "PCD READ STARTED"<<endl;
	PCDReader reader;  
	if (reader.read (PCDfileName, *cloud) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read PCD file \n");
		
	  }
	  cout << "PCD READ:END:Loaded "
				<< cloud->width * cloud->height
				<< " data points from "<<PCDfileName
				<< endl;
}