#include "crow.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <string>
using std::string;
using std::ofstream;
using std::endl;




int main(int argc, char** argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file %s \n", argv[1]);
		return (-1);
	}
	string filePath = argv[2];
	// write File
	ofstream writeFile(filePath.data());
	if( writeFile.is_open() ){
		writeFile << cloud->width * cloud->height << endl;
		for (size_t i = 0; i < cloud->points.size (); ++i) {
			writeFile << cloud->points[i].x << " " << cloud->points[i].y 
			<< " " << cloud->points[i].z << endl;
		}
		writeFile.close();
	}
    crow::SimpleApp app;
    CROW_ROUTE(app,"/")
    ([]{
        return "Hello world";
    });
    app.port(8080).run();
    return 0;

}
