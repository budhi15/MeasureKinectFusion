#include "convertOBJtoPCD.h"

using namespace std;


string convertOBJtoPCD (string pathToObj)
{
	ifstream objFileHandle;
	string pathToPCD;
	unsigned pos = pathToObj.find("."); 
	pathToPCD = pathToObj.substr(0,pos) + ".pcd";

	objFileHandle.open(pathToObj);

	if(!objFileHandle.is_open())
	{
		cerr<<"Unable to open the OBJ file"<<endl;
		 cerr<<"Aborting"<<endl;
		  Sleep(3000);
		  exit(0);
	}
	else
	{
		string tempFileName = "temp";
		ofstream pcdFileHandle;
		fstream tempFileHandle;
		pcdFileHandle.open(pathToPCD);
		tempFileHandle.open(tempFileName, ios::out);
		cout<<"Converting OBJ to PCD"<<endl;
		string headerPart1 = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";
		string headerPart3 = "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n";
		string headerPart5 = "DATA ascii\n";
		unsigned vertexCount = 0;
		unsigned readCount = 0;
		string line;
		string key;
		string x;
		string y;
		string z;
		//currently, this code will only read the vertex data in the OBJ file and break after that. This is for efficiency. It can be extended to read other data as well
		while(getline(objFileHandle, line))
		{
			readCount++;
			istringstream lineParts(line);
			lineParts >> key >> x >> y >> z;
			if(key.compare("#") == 0)
			{
				//comment line in OBJ file. Ignored
			}
			else
			{

				if(key.compare("v") != 0)
				{
					cerr<<"Unreadable line found at line #:"<<readCount<<".Stopped reading OBJ File."<<endl;
					break;
				}
				else
				{
					tempFileHandle<<x<<" "<<y<<" "<<z<<endl;
					vertexCount++;
				}
			}
			
				
		}
		stringstream convert;
		convert<<vertexCount;
		string headerPart2 = "WIDTH " + convert.str() + "\n";
		string headerPart4 = "POINTS " + convert.str() + "\n";
		
		//go to the start of the file to write header
		
		string header = headerPart1 + headerPart2 + headerPart3 + headerPart4 + headerPart5;
		tempFileHandle.close();
		tempFileHandle.open(tempFileName, ios::in);
		//check to see that it exists:
		if (!tempFileHandle.is_open()) {
			//file not found (i.e. it is not opened). Print an error message or do something else.
		}
		else {
		pcdFileHandle<<header;
		pcdFileHandle<<tempFileHandle.rdbuf();
		}
		

		cout<<"PCD file generated."<<endl;
		tempFileHandle.close();
		pcdFileHandle.close();
		objFileHandle.close();
	}

	

	return pathToPCD;
}