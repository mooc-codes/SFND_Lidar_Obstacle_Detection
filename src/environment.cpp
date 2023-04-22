/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    
    // Downsample and region limit the input cloud
    pointProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 5, 1, 1));
    // segment the cloud into ground plane and obstacles
    auto segmentedCloud = pointProcessor->SegmentPlane(inputCloud, 25, 0.3);
    // Cluster the obstacles
    auto obstacles = pointProcessor->Clustering(segmentedCloud.first, 0.53, 20, 500);
    int clusterId = 0;
    
    for(auto cluster : obstacles)
    {
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),Color(1,0,0));
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    // render the ground
    renderPointCloud(viewer, segmentedCloud.second, "Incoming Cloud", Color(0, 1, 0));
}



int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> pcdFiles = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2");
    auto fileName = pcdFiles.begin();

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pointProcessor->loadPcd((*fileName).string());
        cityBlock(viewer, pointProcessor, inputCloud);

        ++fileName;
        if (fileName == pcdFiles.end())
        {
            fileName = pcdFiles.begin();
        }
        
        viewer->spinOnce ();
    } 
}