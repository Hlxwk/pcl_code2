#include<iostream>
#include<string>
#include<vector>
#include<sstream>
#include<stdlib.h>
#include<time.h>


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
    {255,255,0}
};

void printUsage(const char* command)
{
    std::cout << "\n\nUsage: "<<command<<" [options]  [dir of pcd file]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Show pcd file\n"
            << "-s2          Show two pcd files\n"
            << "-ss          Show several pcd files"
            << "-p           cloud_c=cloud_a+cloud_b\n"
            << "-kd          kd tree search\n"
            << "-de          spatial change detection on unorganized point cloud data\n"
            << "-c           Cluster Extraction\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        loadfile2(cloud,argv[2]);
        std::cout<<"Original PointCloud has: "<<cloud->points.size()<<" points."<<std::endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer=showpcd(cloud);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }

    if(pcl::console::find_argument(argc,argv,"-s2")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile1(cloud1,argv[2]);
        loadfile2(cloud2,argv[3]);
        std::cout<<"cloud1 size= "<<cloud1->points.size()<<std::endl
                 <<"cloud2 size= "<<cloud2->points.size()<<std::endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer=showpcd2(cloud1,cloud2);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
        }

    }

    if(pcl::console::find_argument(argc,argv,"-ss")>=0)
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



    if(pcl::console::find_argument(argc,argv,"-c")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        loadfile1(cloud,argv[2]);
        std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; 

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.1f, 0.1f, 0.1f);//default 0.01
        vg.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
        
        
        // Create the segmentation object for the planar model and set all the parameters
        
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_sum(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.06);//default 0.02
        
        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            cloud_plane->clear();
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);


            *cloud_plane_sum += *cloud_plane;


            
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
            
            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }
        // Creating the KdTree object for the search method of the extraction


        pcl::io::savePCDFileASCII("planar.pcd",*cloud_plane_sum);
        



        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.4); // default 0.02
        ec.setMinClusterSize (300);//default 100
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        
        int j = 0;
        
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j++;
        }
    }
    return 0;
}