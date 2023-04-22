#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include "kdtree.h"

void AddNeighbors(int i, KdTree* tree, float distanceTol, std::vector<int>& cluster, std::vector<bool>& isProcessed, const std::vector<std::vector<float>>& points);

std::vector<std::vector<int>> EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif