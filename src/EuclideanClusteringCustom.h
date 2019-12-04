#include <pcl/common/common.h>
#include "KdTreeCustom.h"

void Proximity(int idx, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<bool> &processed, KdTree *tree, float& distanceTol, std::vector<int> &cluster, int maxSize);
std::vector<std::vector<int>> euclideanClusterCustom(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree *tree, float distanceTol, int maxSize);