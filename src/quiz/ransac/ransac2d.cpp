/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <random>
#include <chrono>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	std::vector<float> coefs(3);
	pcl::PointXYZ point;

	const int N = cloud->points.size();
	int maxx = -1;
	float dist;
	
	// For max iterations 
	// Randomly sample subset and fit line
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, (N-1));

    for (int i = 0; i < maxIterations; ++i) {
        std::vector<std::vector<float>> chosen;
		std::unordered_set<int> uniques, closest;

        while (uniques.size() < 2) {
        	int idx = dis(gen);
			point = cloud->points[idx];
			chosen.push_back({point.x, point.y});
			uniques.insert(idx);
        }

		// Fit the line Ax + By + C = 0
		coefs[0] = chosen[0][1] - chosen[1][1]; // A = y1 - y2
		coefs[1] = chosen[1][1] - chosen[1][0]; // B = x2 - x1
		coefs[2] = chosen[0][0] * chosen[1][1] - chosen[1][0] * chosen[0][1]; // C = x1 * y2 - x2 * y1

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
        for (int j = 0; j < N; ++j) {
			dist = std::abs(cloud->points[j].x * coefs[0] + cloud->points[j].y * coefs[1] + coefs[2]);
			dist /= std::sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]); 
            if (dist <= distanceTol) {
                closest.insert(j);
            }
        }

        if (inliersResult.size() < closest.size()) {
            inliersResult = closest;
        }
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	// std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
