// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr ground (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*ground);
    for (int index: inliers->indices)
    {
        ground->points.push_back(cloud->points[index]);
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	pcl::PointIndices::Ptr inlierIndices {new pcl::PointIndices};
    std::unordered_set<int> inliersResult;
	PointT p1, p2, p3, p4;
	float A, B, C, D, dist;
	srand(time(NULL));
	std::unordered_set<int>::iterator itr;
	auto setPoint = [&](int idx, PointT& point)
					{ 
						point = cloud->points[idx];
					};

	auto compute_dist = [&](PointT p4)
						{
										A = ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
								B = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
								C = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x));
								D = -1 * ( (A * p1.x) + (B * p1.y) + (C * p1.z));

								auto numerator = fabs((A * p4.x) + (B * p4.y) + (C * p4.z) + D);
								auto denom = sqrt((A * A) + (B * B) + (C * C));
								return numerator/denom;
						};

	while(maxIterations)
	{

		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}
		

		itr = inliers.begin();
		setPoint(*itr, p1);
		itr++;
		setPoint(*itr, p2);
		itr++;
		setPoint(*itr, p3);
		// std::cout<<p1.x<<p2.x<<p3.x<<std::endl;
		
		for(int idx = 0; idx < cloud->points.size(); idx++)
		{

			if(inliers.count(idx) > 0)
			{
				continue;
			}
			setPoint(idx, p4);


			dist = compute_dist(p4);
			if (dist <= distanceTol)
			{
				inliers.insert(idx);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
		maxIterations--;
	}
	
    for (auto index: inliersResult)
    {
        inlierIndices->indices.emplace_back(index);
    }
    return inlierIndices;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.

    inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr kdTree (new pcl::search::KdTree<PointT>);
    kdTree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusteredIndices;
    typename pcl::EuclideanClusterExtraction<PointT> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance); //cm
    clusterExtraction.setMinClusterSize(minSize);
    clusterExtraction.setMaxClusterSize(maxSize);
    clusterExtraction.setSearchMethod(kdTree);
    clusterExtraction.setInputCloud(cloud);
    clusterExtraction.extract(clusteredIndices);

    for(auto cluster: clusteredIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);

        for (auto idx: cluster.indices)
        {
            cluster_cloud->points.push_back(cloud->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        clusters.push_back(cluster_cloud);

    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

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