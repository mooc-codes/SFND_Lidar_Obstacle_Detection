
#include "cluster.h"

void AddNeighbors(int i, KdTree* tree, float distanceTol, std::vector<int>& cluster, std::vector<bool>& isProcessed, const std::vector<std::vector<float>>& points)
{
	isProcessed[i] = true;
	cluster.push_back(i);
	std::vector<int> neighbors = tree->search(points[i], distanceTol);
	for(int id: neighbors)
	{
		if(!isProcessed[id])
		{
			AddNeighbors(id, tree, distanceTol, cluster, isProcessed, points);
		}
	}
}


std::vector<std::vector<int>> EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> isProcessed(points.size(), false);

	int i = 0;
	while( i < points.size())
	{
		if(isProcessed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		AddNeighbors(i, tree, distanceTol, cluster, isProcessed, points);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;

}

#endif