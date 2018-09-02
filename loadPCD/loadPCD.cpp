#include<iostream>
#include<string>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/console/parse.h>
void printUsage(const char* command)
{
    std::cout << "\n\nUsage: "<<command<<" [options]  [dir of pcd file]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Show pcd file\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}
void loadfile1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char* filename)
{
    pcl::io::loadPCDFile(filename,*cloud);
}
void loadfile2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char* filename)
{
    pcl::PCDReader reader;
    reader.read(filename,*cloud);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> showpcd (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud,255,0,0);//定义点云颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_color,"cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
    //viewer->addCoordinateSystem(1.0);//坐标轴
    viewer->initCameraParameters();
    return viewer;

}
int main(int argc,char** argv)
{
    if(pcl::console::find_argument(argc,argv,"-h")>=0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if(pcl::console::find_argument(argc,argv,"-s")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile2(cloud,argv[2]);
        std::cout<<"Original PointCloud has: "<<cloud->points.size()<<" points."<<std::endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer=showpcd(cloud);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    return 0;
}