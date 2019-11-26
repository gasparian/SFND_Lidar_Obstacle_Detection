#include "EuclidianClusterCustom.h"

void Proximity(int idx, const std::vector<std::vector<float>>& points, std::set<int> &processed, KdTree *tree, float& distanceTol, std::vector<int> &cluster) {

	processed.insert(idx);
	cluster.push_back(idx);

	std::vector<int> nearby = tree->search(points[idx], distanceTol);

	for (int nIdx : nearby) {
		if (!processed.count(nIdx)) {
			Proximity(nIdx, points, processed, tree, distanceTol, cluster);
		}
	}
}

std::vector<std::vector<int>> euclideanClusterCustom(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::set<int> processed;

	for (int idx=0; idx < points.size(); idx++) {
		if (!processed.count(idx)) {
			std::vector<int> cluster;
			Proximity(idx, points, processed, tree, distanceTol, cluster);
			clusters.push_back(cluster);
		}
	}

	return clusters;

}