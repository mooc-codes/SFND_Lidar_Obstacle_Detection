#ifndef SEG_PCL_COMMON
#define SEG_PCL_COMMON
#include <pcl/common/common.h>
#endif
#include <unordered_set>
template<typename PointT>
pcl::PointIndices::Ptr RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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