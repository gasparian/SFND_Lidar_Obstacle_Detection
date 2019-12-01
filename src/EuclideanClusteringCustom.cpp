#include "EuclideanClusteringCustom.h"

void Proximity(int idx, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::set<int> &processed, KdTree *tree, float& distanceTol, std::vector<int> &cluster, int maxSize) 
{

	processed.insert(idx);
	cluster.push_back(idx);

	std::vector<float> point = {cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
	std::vector<int> nearby = tree->search(point, distanceTol);
	
	for (int nIdx : nearby) {
		if ( (!processed.count(nIdx)) && (cluster.size() < maxSize) ) {
			Proximity(nIdx, cloud, processed, tree, distanceTol, cluster, maxSize);
		}
	}
}

std::vector<std::vector<int>> euclideanClusterCustom(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree* tree, float distanceTol, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::set<int> processed;

	for (int idx=0; idx < cloud->points.size(); idx++) {
		if ( !processed.count(idx) ) {
			std::vector<int> cluster;
			Proximity(idx, cloud, processed, tree, distanceTol, cluster, maxSize);
			clusters.push_back(cluster);
		}
	}

	return clusters;

}