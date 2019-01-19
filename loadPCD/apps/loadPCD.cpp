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
            << "-s           Show several pcd files\n"
            << "-p           cloud_c=cloud_a+cloud_b\n"
            << "-kd          kd tree search\n"
            << "-de          spatial change detection on unorganized point cloud data\n"
            << "-c           Cluster Extraction\n"
            << "-ndt         Registration using NDT(one thread)\n"
            << "-icp1        Registration using icp(point-to-plane-1)\n"
            << "-gicp        Registration using icp(plane-to-plane)\n"
            << "-icp2        Registration using icp(point-to-plane-2)\n"
            << "-ndt_omp_t   Registration using NDT(test multiple threads and search methods)\n"
            << "-ndt_omp     Registration using NDT(multiple threads and different seach methods)\n"
            << "-las         Las to pcd\n"
            << " -i          Intensity\n"
            << "-t           Test for pointcloud\n"
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

    // if(pcl::console::find_argument(argc,argv,"-s")>=0)
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //     loadfile2(cloud,argv[2]);
    //     std::cout<<"Original PointCloud has: "<<cloud->points.size()<<" points."<<std::endl;
    //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //     viewer=showpcd(cloud);
    //     while(!viewer->wasStopped())
    //     {
    //         viewer->spinOnce();
    //     }
    // }

    // if(pcl::console::find_argument(argc,argv,"-s2")>=0)
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
    //                                         cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    //     loadfile1(cloud1,argv[2]);
    //     loadfile2(cloud2,argv[3]);
    //     std::cout<<"cloud1 size= "<<cloud1->points.size()<<std::endl
    //              <<"cloud2 size= "<<cloud2->points.size()<<std::endl;
    //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //     viewer=showpcd2(cloud1,cloud2);
    //     while(!viewer->wasStopped())
    //     {
    //         viewer->spinOnce();
    //     }

    // }

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
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                            cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(argv[2],*cloud);
        std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; 

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
        // vg.setInputCloud (cloud);
        // vg.setLeafSize (0.1f, 0.1f, 0.1f);//default 0.01
        // vg.filter (*cloud_filtered);
        // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
        
        *cloud_filtered=*cloud;
        // Create the segmentation object for the planar model and set all the parameters
        
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane_sum(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.06);//default 0.02
        
        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points)//default 0.3
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
            pcl::ExtractIndices<pcl::PointXYZI> extract;
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
        



        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud (cloud_filtered);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (0.3); // default 0.02
        ec.setMinClusterSize (600);//default 300
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        
        int j = 0;
        
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            //writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false); 
            pcl::io::savePCDFileASCII(ss.str(),*cloud_cluster);
            j++;
        }
    }





    if(pcl::console::find_argument(argc,argv,"-ndt")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        loadfile1(target_cloud,argv[2]);
        loadfile2(input_cloud,argv[3]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(0.2,0.2,0.2);
        approximate_voxel_filter.setInputCloud(input_cloud);
        approximate_voxel_filter.filter(*cloud_filtered);

        pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(0.1);
        ndt.setResolution(1.0);

        ndt.setMaximumIterations(35);
        ndt.setInputSource(cloud_filtered);
        ndt.setInputTarget(target_cloud);

        Eigen::Matrix4f init_guess;
        Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        init_guess = (init_translation * init_rotation).matrix ();

        init_guess = Eigen::Matrix4f::Identity();

        // init_guess<<0.753145,-0.657489,0.0219242,2.01461,
        //             0.657332,0.753457,0.014739,0.0717241,
        //             -0.0262097,0.00331083,0.999651,0.0253123,
        //             0,0,0,1;
        init_guess<<0.627894,-0.750407,-0.206491 ,-10.0237,
                    0.741192 ,0.495585 ,0.452803 ,-7.56973,
                    -0.237452,-0.437362  ,0.86737 ,-6.17818,
                    0        ,0        ,0        ,1;



        clock_t starttime,endtime;
        starttime=clock();



        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align (*output_cloud, init_guess);

        endtime=clock();
        
        pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

        std::cerr<<"Registration time = "<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<"s"<<std::endl;
        std::cout<<ndt.getFinalTransformation()<<endl;
        
        pcl::io::savePCDFileASCII ("room_scan2_transformed_ndt.pcd", *output_cloud);
        pcl::io::savePCDFileASCII ("plus.pcd",*output_cloud+*target_cloud);

    }

    if(pcl::console::find_argument(argc,argv,"-icp1")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        loadfile1(cloud1,argv[3]);
        loadfile1(cloud2,argv[2]);
        

        pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*cloud1,*src);
        pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*cloud2,*tgt);


        pcl::NormalEstimation<pcl::PointNormal,pcl::PointNormal> norm_est;
        norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
        norm_est.setKSearch(5);
        norm_est.setInputCloud(tgt);
        norm_est.compute(*tgt);

        pcl::IterativeClosestPoint<pcl::PointNormal,pcl::PointNormal> icp;
        typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal> pointtoplane;

        boost::shared_ptr<pointtoplane> point_to_plane(new pointtoplane);
        icp.setTransformationEstimation(point_to_plane);

        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        icp.setRANSACIterations(20);
        icp.setMaximumIterations(35);
        icp.setTransformationEpsilon(1e-2);

        Eigen::Matrix4f init_guess;
        
        pcl::PointCloud<pcl::PointNormal> output_cloud;
        // init_guess<<0.753145,-0.657489,0.0219242,2.01461,
        //             0.657332,0.753457,0.014739,0.0717241,
        //             -0.0262097,0.00331083,0.999651,0.0253123,
        //             0,0,0,1;
        // Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        // init_guess = (init_translation * init_rotation).matrix ();
        // init_guess=Eigen::Matrix4f::Identity();
        Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        init_guess = (init_translation * init_rotation).matrix ();
        icp.align(output_cloud,init_guess);
        cout<<init_guess<<endl;


        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud1,*output,icp.getFinalTransformation());
        //std::cout<<icp.getFinalTransformation()<<endl;
        pcl::io::savePCDFileASCII ("room_scan2_transformed_icp.pcd", *output);
    }



    if(pcl::console::find_argument(argc,argv,"-gicp")>=0)
    {
        using namespace std;
        pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

        loadfile1(tgt,argv[2]);
        loadfile1(src,argv[3]);


        string s=argv[3];
        int start=s.find_last_of('/');
        int end=s.find_last_of('.');
        string file=s.substr(start+1,end-start-1);
        std::stringstream filename;
        filename<<file<<"_transformed_gicp.pcd";

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
        
        gicp.setRotationEpsilon(1e-9);
        gicp.setTransformationEpsilon(1e-9);
        gicp.setMaxCorrespondenceDistance(2);
        gicp.setMaximumOptimizerIterations(50);
        gicp.setEuclideanFitnessEpsilon(0.5);

        gicp.setInputTarget(tgt);
        gicp.setInputSource(src);


        Eigen::Matrix4f init_guess;
        Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        init_guess = (init_translation * init_rotation).matrix ();
        init_guess = Eigen::Matrix4f::Identity();
        // init_guess<<0.753145,-0.657489,0.0219242,2.01461,
        //             0.657332,0.753457,0.014739,0.0717241,
        //             -0.0262097,0.00331083,0.999651,0.0253123,
        //             0,0,0,1;

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI> unused_result;

        clock_t starttime,endtime;
        starttime=clock();
        gicp.align(*output, init_guess);
        endtime=clock();


        pcl::transformPointCloud(*src,*output,gicp.getFinalTransformation());

        std::cerr<<"Registration time = "<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<"s"<<std::endl;
        cout<<gicp.getFinalTransformation()<<endl;
        //std::cout<<icp.getFinalTransformation()<<endl;
        pcl::io::savePCDFileASCII (filename.str(), *output);
    }


    if(pcl::console::find_argument(argc,argv,"-icp2")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile(argv[2],*tgt);
        pcl::io::loadPCDFile(argv[3],*src);

        std::string s=argv[3];
        int start=s.find_last_of('/');
        int end=s.find_last_of('.');
        std::string file=s.substr(start+1,end-start-1);
        std::stringstream filename;
        filename<<file<<"_transformed_icp2.pcd";
        

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_trans_normals(new pcl::PointCloud<pcl::PointNormal>);

        add_normals(src,cloud_source_normals);
        add_normals(tgt,cloud_target_normals);

        pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal>);

        icp->setTransformationEpsilon(0.0000000001);//default 0.0000000001
        icp->setMaxCorrespondenceDistance(2.0);
        icp->setMaximumIterations(50);
        icp->setRANSACIterations(20);
        icp->setInputSource(cloud_source_normals); //
        icp->setInputTarget(cloud_target_normals);
        
        Eigen::Matrix4f init_guess;
        Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        init_guess = (init_translation * init_rotation).matrix ();
        init_guess = Eigen::Matrix4f::Identity();

        // init_guess<<0.753145,-0.657489,0.0219242,2.01461,
        //             0.657332,0.753457,0.014739,0.0717241,
        //             -0.0262097,0.00331083,0.999651,0.0253123,
        //             0,0,0,1;

        clock_t starttime,endtime;
        starttime=clock();
        icp->align(*cloud_source_trans_normals, init_guess);
        endtime=clock();

        pcl::transformPointCloud(*cloud_source_normals,*cloud_source_trans_normals,icp->getFinalTransformation());

        std::cerr<<"Registration time = "<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<"s"<<std::endl;
        cout<<icp->getFinalTransformation()<<endl;

        pcl::io::savePCDFileASCII(filename.str(), *cloud_source_trans_normals);

    }


    if(pcl::console::find_argument(argc,argv,"-ndt_omp_t")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());


        loadfile1(target_cloud,argv[2]);
        loadfile1(source_cloud,argv[3]);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
        
        
        voxelgrid.setInputCloud(target_cloud);
        voxelgrid.filter(*downsampled);
        *target_cloud = *downsampled;
        
        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;
        
        
        std::cout << "--- pcl::NDT ---" << std::endl;
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt->setResolution(1.0);
        ndt->setTransformationEpsilon(0.01);
        ndt->setStepSize(0.1);

        ndt->setMaximumIterations(35);

        
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(ndt, target_cloud, source_cloud);
        
        std::vector<int> num_threads = {1, omp_get_max_threads()};
        std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
            {"KDTREE", pclomp::KDTREE},
            {"DIRECT7", pclomp::DIRECT7},
            {"DIRECT1", pclomp::DIRECT1}
        };
        
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_omp->setResolution(1.0);
        for(int n : num_threads) {
            for(const auto& search_method : search_methods) {
                std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---" << std::endl;
                ndt_omp->setNumThreads(n);
                ndt_omp->setNeighborhoodSearchMethod(search_method.second);
                ndt_omp->setTransformationEpsilon(0.01);
                ndt_omp->setStepSize(0.1);
                ndt_omp->setMaximumIterations(35);
                aligned = align(ndt_omp, target_cloud, source_cloud);
                }
        }
    }

    if(pcl::console::find_argument(argc,argv,"-ndt_omp")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

        loadfile1(tgt,argv[2]);
        loadfile1(src,argv[3]);
        std::string s=argv[3];
        int start=s.find_last_of('/');
        int end=s.find_last_of('.');
        std::string file=s.substr(start+1,end-start-1);
        std::stringstream filename;
        filename<<file<<"_transformed_ndt_omp.pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);
        
        
        voxelgrid.setInputCloud(tgt);
        voxelgrid.filter(*downsampled);
        *tgt = *downsampled;
        
        voxelgrid.setInputCloud(src);
        voxelgrid.filter(*downsampled);
        *src = *downsampled;

        pclomp::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt_omp;

        ndt_omp.setInputSource(src);
        ndt_omp.setInputTarget(tgt);

        ndt_omp.setNumThreads(omp_get_max_threads());
        ndt_omp.setResolution(1.0);
        ndt_omp.setNeighborhoodSearchMethod(pclomp::NeighborSearchMethod::DIRECT1);
        ndt_omp.setTransformationEpsilon(0.01);
        ndt_omp.setStepSize(1);//default 0.1
        ndt_omp.setMaximumIterations(100);//default 35


        Eigen::Matrix4f init_guess;
        // Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
        // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
        // init_guess = (init_translation * init_rotation).matrix ();
        init_guess = Eigen::Matrix4f::Identity();

        
        clock_t starttime,endtime;
        starttime=clock();

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        ndt_omp.align(*output,init_guess);
        
        endtime=clock();

        std::cerr<<"Registration time = "<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<"s"<<std::endl;

        std::cout<<ndt_omp.getFinalTransformation()<<endl;


        pcl::transformPointCloud(*src,*output,ndt_omp.getFinalTransformation());

        pcl::io::savePCDFileASCII(filename.str(), *output);

    }



    if(pcl::console::find_argument(argc,argv,"-las")>=0)
    {
        std::ifstream ifs;
        ifs.open(argv[2],std::ios::in|std::ios::binary);

        std::string s=argv[2],output_dir;
        int start=s.find_last_of('/');
        int end=s.find('.');
        //cout<<start<<" "<<end<<endl;
        output_dir=s.substr(start+1,end-start-1);
        std::stringstream ss;
        ss<<output_dir<<".pcd";
        cout<<ss.str()<<endl;

        liblas::ReaderFactory readerFactory;
        liblas::Reader reader=readerFactory.CreateWithStream(ifs);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

        while(reader.ReadNextPoint())
        {
            double x=reader.GetPoint().GetX();
            double y=reader.GetPoint().GetY();
            double z=reader.GetPoint().GetZ();
            pcl::PointXYZ p(x,y,z);
            cloud_output->push_back(p);
        }

        cloud_output->width=cloud_output->points.size();
        cloud_output->height=1;
        cloud_output->is_dense=false;
        pcl::io::savePCDFileASCII(ss.str(),*cloud_output);

    }

     if(pcl::console::find_argument(argc,argv,"-i")>=0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

        pcl::io::loadPCDFile(argv[2],*cloud);

        int point_number = cloud->points.size();
        std::cerr<<"point number = "<<point_number<<std::endl;

        std::set<float> intensity;
        for(int i=0;i<point_number;++i)
        intensity.insert(cloud->points[i].intensity);

        cout<<intensity.size()<<endl;

    }

    if(pcl::console::find_argument(argc,argv,"-t")>=0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        loadfile1(cloud,argv[2]);

        Eigen::Matrix4f trans;
        trans<< 0.333592 ,-0.941918,-0.0388056  ,-10.2338,
                0.137894 ,0.0894748 ,-0.986397  ,-7.94611,
                0.932578 ,0.323704  ,0.159733   ,-6.3033,
                0        ,0         ,0          ,1;
        trans<< 0.627894,-0.750407,-0.206491 ,-10.0237,
                    0.741192 ,0.495585 ,0.452803 ,-7.56973,
                    -0.237452,-0.437362  ,0.86737 ,-6.17818,
                    0        ,0        ,0        ,1;


        pcl::transformPointCloud(*cloud,*output,trans);
        pcl::io::savePCDFileASCII("test.pcd",*output);

    }
    return 0;
}