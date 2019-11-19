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
	float A, B, C;

	const int N = cloud->points.size();
	int maxx = -1;
	float dist;
	
	// For max iterations 
	// Randomly sample subset and fit line
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, (N-1));

    for (int i = 0; i < maxIterations; ++i) {
        std::vector<pcl::PointXYZ> chosen;
		std::unordered_set<int> uniques, closest;

        while (uniques.size() < 2) {
        	int idx = dis(gen);
			chosen.push_back(cloud->points[idx]);
			uniques.insert(idx);
        }

		// Fit the line Ax + By + C = 0
		A = chosen[0].y - chosen[1].y; // A = y1 - y2
		B = chosen[1].x - chosen[0].x; // B = x2 - x1
		C = chosen[0].x * chosen[1].y - chosen[1].x * chosen[0].y; // C = x1 * y2 - x2 * y1

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
        for (int j = 0; j < N; ++j) {
			dist = std::abs(A * cloud->points[j].x + B * cloud->points[j].y + C);
			dist /= std::sqrt(A * A + B * B); 
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	float A, B, C, D;
	float x1, x2;
	float y1, y2;
	float z1, z2;

	const int N = cloud->points.size();
	int maxx = -1;
	float dist;
	
	// For max iterations 
	// Randomly sample subset and fit line
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, (N-1));

    for (int i = 0; i < maxIterations; ++i) {
		std::vector<pcl::PointXYZ> chosen;
		std::unordered_set<int> uniques, closest;

        while (uniques.size() < 3) {
        	int idx = dis(gen);
			chosen.push_back(cloud->points[idx]);
			uniques.insert(idx);
        }

		// Fit the line Ax + By + Cz + D = 0
		x1 = chosen[1].x - chosen[0].x; // _x2 - _x1
		y1 = chosen[1].y - chosen[0].y; // _y2 - _y1
		z1 = chosen[1].z - chosen[0].z; // _z2 - _z1

		x2 = chosen[2].x - chosen[0].x; // _x3 - _x1
		y2 = chosen[2].y - chosen[0].y; // _y3 - _y1
		z2 = chosen[2].z - chosen[0].z; // _z3 - _z1

		A = y1 * z2 - y2 * z1;
		B = x1 * z1 - x2 * z1;
		C = x1 * y2 - x2 * y1;
		D = -1.0 * (A * chosen[0].x + B * chosen[0].y + C * chosen[0].z); // -(A * _x1 + B * _y1 + C * _z1)

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
        for (int j = 0; j < N; ++j) {
			dist = std::abs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D);
			dist /= std::sqrt(A * A + B * B + C * C); 
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
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
