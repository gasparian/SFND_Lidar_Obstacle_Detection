#include <vector>
#include <algorithm>
#include <unordered_set>
#include <pcl/common/common.h>

std::vector<float> get_coefs(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::unordered_set<int> inliersResult);
void fill_inliers(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<float> coefs, std::unordered_set<int>& closest, float distanceThreshold);
