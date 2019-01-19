#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <vector>
#include <string>
#include <iterator>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <pcl/PCLPointCloud2.h>
using namespace std;

int main(int argc, char const **argv)
{
    // vector<string> file_lists;
    // struct dirent *ptr;
    // DIR *dir;
    // string filename=argv[1];
    // dir = opendir(filename.c_str());
    // while( (ptr=readdir(dir)) != NULL )
	// {
	// 	// skip '.' and '..'
	// 	if(ptr->d_name[0] == '.')
	// 		continue;
		
	// 	// skip the files that we are not need 
		
	// 	std::string _filename = ptr->d_name;
		
	// 	// get full path
	// 	file_lists.push_back( (filename+'/'+ptr->d_name).c_str() );
	// }
    //copy(file_lists.begin(),file_lists.end(),ostream_iterator<string>(cout,"\n"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud;
    // for(string s:file_lists)
    // {
    //     pcl::io::loadPCDFile(s,cloud);
    //     pcl::io::loadPCDFile(s,*cloud1);
    //     for(int i=0;i<5;i++)
    //     std::cout<<cloud1->points[i].x<<" "
    //              <<cloud1->points[i].y<<" "
    //              <<cloud1->points[i].z<<std::endl;
    //     std::cout<<s<<"  "<<endl;
    //     pcl::PLYWriter writer;
    //     pcl::PLYReader read;
    //     string temp=s.substr(0,s.size()-2)+"ly";
    //     //std::cout<<temp<<std::endl;
    //     writer.write(temp,cloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),true,true);
    //     read.read(temp,*cloud1);
    // }
    pcl::PLYReader reader;
    std::cout<<argv[1]<<std::endl;
    reader.read(argv[1],*cloud1);
    //pcl::fromPCLPointCloud2(cloud,*cloud1);
        for(int i=0;i<5;i++)
        std::cout<<cloud1->points[i].x<<" "
                 <<cloud1->points[i].y<<" "
                 <<cloud1->points[i].z<<std::endl;
    std::cout<<cloud1->points.size()<<std::endl;
    return 0;
    

}
