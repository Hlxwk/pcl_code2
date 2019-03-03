#include<iostream>
#include<algorithm>
#include<string>
#include<vector>
#include<sstream>
#include<stdlib.h>
#include<time.h>
#include<ctime>
#include<set>

#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>

#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/console/parse.h>
#include<pcl-1.7/pcl/kdtree/kdtree_flann.h>
#include<pcl-1.7/pcl/octree/octree.h>
#include<pcl-1.7/pcl/point_cloud.h>
#include<pcl-1.7/pcl/ModelCoefficients.h>
#include<pcl-1.7/pcl/filters/extract_indices.h>
#include<pcl-1.7/pcl/filters/voxel_grid.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/kdtree/kdtree.h>
#include<pcl-1.7/pcl/sample_consensus/method_types.h>
#include<pcl-1.7/pcl/sample_consensus/model_types.h>
#include<pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include<pcl-1.7/pcl/segmentation/extract_clusters.h>


#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/registration/ndt.h>
#include<pcl-1.7/pcl/filters/approximate_voxel_grid.h>
#include<boost/thread/thread.hpp>
#include<pcl-1.7/pcl/registration/icp.h>
#include<pcl-1.7/pcl/registration/gicp.h>
#include<pcl-1.7/pcl/registration/transformation_estimation_point_to_plane.h>


#include<pclomp/ndt_omp.h>

#include <fstream>
#include<liblas/liblas.hpp>



struct color_value{
    int r;
    int g;
    int b;
};
const color_value color[]={
    {255,255,255},
    {255,0,0},
    {0,255,0},
    {0,0,255},
    {218,112,214},
    {255,0,255},
    {255,255,0},
    {160,82,45},
    {8,46,84}
};

void printUsage(const char* command)
{
    std::cout << "\n\nUsage: "<<command<<" [options]  [dir of pcd file]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           pcd visualization"
            << "-c           convert bin to pcd\n"
            << "\n\n";
}



void loadfile1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char* filename)//11
{
    pcl::io::loadPCDFile(filename,*cloud);
}

void loadfile2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char* filename)
{
    pcl::PCDReader reader;
    reader.read(filename,*cloud);
}


void add_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud(cloud);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(searchTree);
    ne.setKSearch(15);
    ne.compute(*normals);

    pcl::concatenateFields(*cloud,*normals,*cloud_with_normals);
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> showpcd (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud,255,255,255);//定义点云颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_color,"cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud");
    //viewer->addCoordinateSystem(1.0);//坐标轴
    viewer->initCameraParameters();
    return viewer;

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> showpcd2 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1,255,255,255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud1,cloud1_color,"cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud2,cloud2_color,"cloud2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud2");
    viewer->initCameraParameters();
    return viewer;
}



// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud ) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  clock_t starttime,endtime1,endtime2;
  starttime=clock();
  registration->align(*aligned);
  endtime1=clock();
  std::cout << "single : " << (double)(endtime1-starttime)/CLOCKS_PER_SEC<<"s"<< std::endl;

  for(int i=0; i<10; i++) {
    registration->align(*aligned);
  }
  endtime2=clock();
  std::cout << "multiple : " << (double)(endtime2-endtime1)/CLOCKS_PER_SEC<<"s"<< std::endl;


  return aligned;
}


int main(int argc,char** argv)
{
    srand((unsigned)time(NULL));



    if(pcl::console::find_argument(argc,argv,"-h")>=0)
    {
        printUsage(argv[0]);
        //return 0;
    }



    if(pcl::console::find_argument(argc,argv,"-s")>=0)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        viewer->setBackgroundColor(0,0,0);
        for(int i=2;i<argc;++i)
        {
            cloud->clear();
            loadfile1(cloud,argv[i]);
            std::stringstream ss;
            ss<<"cloud"<<i-1;
            //int r=rand()%256,g=rand()%256,b=rand()%256;

            int r=color[i-2].r;
            int g=color[i-2].g;
            int b=color[i-2].b;


            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,r,g,b);
            viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_color,ss.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,ss.str());
        }
        viewer->initCameraParameters();
        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }


    if(pcl::console::find_argument(argc,argv,"-c")>=0)
    {
        std::fstream input(argv[2], std::ios::in |std::ios::binary);
        if(!input.good())
        {
            std::cerr<<"could not read file : "<<argv[2]<<std::endl;
            exit(0);
        }

        using namespace std;
        string input_filename=argv[2];
        int sub_len=input_filename.find('.');
        int start_sub=input_filename.rfind('/');
        string output_filename=input_filename.substr(start_sub+1,sub_len-start_sub);
        //std::cout<<output_filename<<std::endl;
        output_filename+=".pcd";

        input.seekg(0,std::ios::beg);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;input.good()&&!input.eof();i++)
        {
            pcl::PointXYZI point;
            input.read((char*) &point.x,3*sizeof(float));//4 for only x,y,z no intensity
            input.read((char*) &point.intensity,sizeof(float));
            cloud->push_back(point);
        }
        input.close();

        cloud->width=cloud->points.size();
        cloud->height=1;
        cloud->is_dense=false;

        pcl::io::savePCDFileASCII(output_filename,*cloud);

    }


      

   
    return 0;
}
    
    