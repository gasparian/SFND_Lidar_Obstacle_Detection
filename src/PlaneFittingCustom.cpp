#include "PlaneFittingCustom.h"

void fill_inliers(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<float> coefs, std::unordered_set<int>& closest, float distanceThreshold) {
    int N = cloud->points.size();
    float dist;
    for (int j = 0; j < N; ++j) {
        dist = std::abs(coefs[0] * cloud->points[j].x + coefs[1] * cloud->points[j].y + coefs[2] * cloud->points[j].z + coefs[3]);
        dist /= std::sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1] + coefs[2] * coefs[2]); 
        if (dist <= distanceThreshold) {
            closest.insert(j);
        }
    }
}

std::vector<float> get_coefs(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::unordered_set<int> inliersResult) {

    float summ[3] = {0.0, 0.0, 0.0}, centroid[3];
    const int N = inliersResult.size();

    for (auto idx=inliersResult.begin(); idx!=inliersResult.end(); ++idx) {
        std::vector<float> point = {cloud->points[*idx].x, cloud->points[*idx].y, cloud->points[*idx].z};
        for (int c = 0; c < 3; ++c) {
            summ[c] += point[c];
        }
    }

    float k = 1.0 / N;
    for (int c = 0; c < 3; ++c) {
        centroid[c] = summ[c] * k;
    }

    float xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
    std::vector<float> r;
    for (auto idx=inliersResult.begin(); idx!=inliersResult.end(); ++idx) {
        r = {cloud->points[*idx].x - centroid[0], 
             cloud->points[*idx].y - centroid[1], 
             cloud->points[*idx].z - centroid[2]};
        xx += r[0] * r[0];
        xy += r[0] * r[1];
        xz += r[0] * r[2];
        yy += r[1] * r[1];
        yz += r[1] * r[2];
        zz += r[2] * r[2];
    }

    float det_x = yy * zz - yz * yz;
    float det_y = xx * zz - xz * xz;
    float det_z = xx * yy - xy * xy;

    float det_max = std::max({det_x, det_y, det_z});

    std::vector<float> dirr;
    if (det_max == det_x) {
        dirr = {det_x, xz*yz - xy*zz, xy*yz - xz*yy};
    } else if (det_max == det_y) {
        dirr = {xz*yz - xy*zz, det_y, xy*xz - yz*xx};
    } else {
        dirr = {xy*yz - xz*yy, xy*xz - yz*xx, det_z};
    }

    if (dirr[0]+dirr[1]+dirr[2] == 0.0) {
    	return {};
    }

    // make a unit normal vector
    float dirr_norm = 0.0;
    for (const auto& value : dirr) {
        dirr_norm += value * value;
    }
    dirr_norm = std::sqrt(dirr_norm);

    int c = 0;
    float d = 0.0;
    std::vector<float> point = {cloud->points[0].x, cloud->points[0].y, cloud->points[0].z};
    for (auto& value : dirr) {
        value = value / dirr_norm;
        d += -1.0 * value * point[c];
        c++;
    }
    dirr.push_back(d);

    return dirr;
}