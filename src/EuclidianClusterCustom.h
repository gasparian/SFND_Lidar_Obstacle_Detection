#include <set>
#include <vector>
#include <algorithm>
#include "quiz/cluster/kdtree.h"

std::vector<std::vector<int>> euclideanClusterCustom(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);
void Proximity(int idx, const std::vector<std::vector<float>>& points, std::set<int> &processed, KdTree *tree, float& distanceTol, std::vector<int> &cluster);
