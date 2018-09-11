#include<iostream>
#include<string>
#include<vector>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/console/parse.h>
#include<pcl-1.7/pcl/kdtree/kdtree_flann.h>
#include<pcl-1.7/pcl/octree/octree.h>
#include<pcl-1.7/pcl/point_cloud.h>
void printUsage(const char* command)
{
    std::cout << "\n\nUsage: "<<command<<" [options]  [dir of pcd file]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Show pcd file\n"
            << "-p           cloud_c=cloud_a+cloud_b\n"
            << "-kd          kd tree search\n"
            << "-de          spatial change detection on unorganized point cloud data\n"
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
        //return 0;
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
    if(pcl::console::find_argument(argc,argv,"-p")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_b(new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_c(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile1(cloud_a,argv[2]);
        loadfile1(cloud_b,argv[3]);
        *cloud_c=*cloud_a;
        *cloud_c+=*cloud_b;
        std::cout<<"Plused PointCloud has: "<<cloud_c->points.size()<<" points."<<std::endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer=showpcd(cloud_c);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }  
    if(pcl::console::find_argument(argc,argv,"-kd")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile1(cloud,argv[2]);
        pcl::PointXYZ searchPoint;
        searchPoint=cloud->points[1000];//1000(random)
        int K(10);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                          << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                          << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
        std::cout << "Neighbors within radius search at (" << searchPoint.x 
                                                    << " " << searchPoint.y 
                                                    << " " << searchPoint.z
                                                    << ") with radius=" << radius << std::endl;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                          << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                          << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }


    }
    if(pcl::console::find_argument(argc,argv,"-de")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile1(cloud_a,argv[2]);
        loadfile2(cloud_b,argv[3]);
        float resolution=32.0f;
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud(cloud_a);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();
        octree.setInputCloud(cloud_b);
        octree.addPointsFromInputCloud();
        std::vector<int> newPointIdxVector;
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);
        std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
        for (size_t i = 0; i < newPointIdxVector.size (); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloud_b->points[newPointIdxVector[i]].x << " "
              << cloud_b->points[newPointIdxVector[i]].y << " "
              << cloud_b->points[newPointIdxVector[i]].z << std::endl;

    }
    return 0;
}