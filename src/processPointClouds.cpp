// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "utils/segment.h"
#include "utils/cluster.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sampler;
    sampler.setInputCloud(cloud);
    sampler.setLeafSize(filterRes, filterRes, filterRes);
    sampler.filter(*cloud);

    pcl::CropBox<PointT> boundedRegion;
    boundedRegion.setMin(minPoint);
    boundedRegion.setMax(maxPoint);
    boundedRegion.setInputCloud(cloud);
    boundedRegion.filter(*cloud);

    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roofBox;
    roofBox.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roofBox.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roofBox.setInputCloud(cloud);
    roofBox.filter(roofIndices); 

    pcl::PointIndices::Ptr roofPoints {new pcl::PointIndices};
    for(auto point: roofIndices)
    {
        roofPoints->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> roofExtractor;
    roofExtractor.setInputCloud(cloud);
    roofExtractor.setIndices(roofPoints);
    roofExtractor.setNegative(true);
    roofExtractor.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
 
    typename pcl::PointCloud<PointT>::Ptr ground {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr obstacles {new pcl::PointCloud<PointT>};
    
    for (int index: inliers->indices)
    {
        ground->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Convert point cloud into std::vector<std::vector<float>>
    std::vector<std::vector<float>> points;
    for( PointT point: cloud->points)
    {
        points.push_back(std::vector<float>{point.x, point.y, point.z});
    }

    // Make Kdtree and insert the points
    KdTree* tree = new KdTree();
    for(int i=0; i < points.size(); i++)
    {
        tree->insert(points[i], i);
    }

    // Do euclidean clustering to get the obstacle cluster indices
    auto clusterIdxs = EuclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
   
    // Make indices to point cloud
    std::vector<typename pcl::PointCloud<PointT>::Ptr> obstacles;
    for (auto cluster: clusterIdxs)
    {
        typename pcl::PointCloud<PointT>::Ptr obstacle{new pcl::PointCloud<PointT>};
        for (int idx: cluster)
        {
            PointT p;
            p.x = points[idx][0];
            p.y = points[idx][1];
            p.z = points[idx][2];
            obstacle->points.push_back(p);
        }
        obstacles.push_back(obstacle);
    }

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return obstacles;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}