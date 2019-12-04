// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "EuclideanClusteringCustom.h"
#include "PlaneFittingCustom.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {} 


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float minDistance, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloudCropped (new pcl::PointCloud<PointT> ());

    // Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloudFiltered);
    boxFilter.filter(*cloudCropped);

    // // Manual filtering points from the car's roof (euclidean distance)
    // typename pcl::PointCloud<PointT>::Ptr cloudFilteredRoof (new pcl::PointCloud<PointT> ());
    // for (int i=0; i < cloudCropped->points.size(); i++) {
    //     float dist = std::sqrt(cloudCropped->points[i].x * cloudCropped->points[i].x + 
    //                            cloudCropped->points[i].y * cloudCropped->points[i].y + 
    //                            cloudCropped->points[i].z * cloudCropped->points[i].z);
    //     if ( dist > minDistance ) {
    //         cloud_filtered_roof->points.push_back(cloudCropped->points[i]);
    //     }
    // }

    //////////////////////////////////
    // filter points with PCL methods
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    // set known box of the ego car
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudCropped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudCropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudCropped);
    //////////////////////////////////

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudCropped;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>), obstCloud (new pcl::PointCloud<PointT>);
    // Extract the inliers

    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    // Print below works if filter planeCloud via `ExtractIndices` method
    // std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Create the filtering object
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
        
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // Fill in this function to find inliers for the cloud.
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    } 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool finalFitting)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    if ( maxIterations < 0 ) {
        // https://en.wikipedia.org/wiki/Random_sample_consensus#Parameters
        // w = inliers / all_opints
        float w_n = 0.6 * 0.6 * 0.6; // probability that all three points are inliers
        float p = 0.99; //  desired probability
        maxIterations = std::log(1 - p) / std::log(1 - w_n) + 1;
    }

    std::unordered_set<int> inliersResult;
	float A, B, C, D;
	float x1, x2;
	float y1, y2;
	float z1, z2;

	const int N = cloud->points.size();
	int maxx = -1;
	
	// For max iterations 
	// Randomly sample subset and fit line
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, (N-1));

    for (int i = 0; i < maxIterations; ++i) {
		std::vector<PointT> chosen;
		std::unordered_set<int> uniques, closest;

        while (uniques.size() < 3) {
        	int idx = dis(gen);
			chosen.push_back(cloud->points[idx]);
			uniques.insert(idx);
        }

		// Fit the plane Ax + By + Cz + D = 0
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
        std::vector<float> coefs = {A, B, C, D};
        fill_inliers(cloud, coefs, closest, distanceThreshold);

        if (inliersResult.size() < closest.size()) {
            inliersResult = closest;
        }
	}

    if (inliersResult.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    } 

    if ( finalFitting ) {
        // final plane fitting using ordinary regression on inliers list
        std::vector<float> coefs = get_coefs(cloud, inliersResult);
        std::unordered_set<int> inliersResult;
        fill_inliers(cloud, coefs, inliersResult, distanceThreshold);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    segResult = std::make_pair(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


// http://pointclouds.org/documentation/tutorials/cluster_extraction.php
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance(clusterTolerance); // in cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndeces : cluster_indices) {

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (int index : getIndeces.indices)
            cloud_cluster->points.push_back(cloud->points[index]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool medianBalancing)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;

    if ( medianBalancing ) {

        // Median balanced KD-tree
        // add a index to the vector of points to build a tree
        std::vector<std::pair<int, std::vector<float>>> pointsIdx(cloud->points.size());
        for (int i = 0; i < cloud->points.size(); ++i) { 
            std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            pointsIdx[i] = std::make_pair(i, point);
        } 
        tree->insertBalanced(tree->root, pointsIdx, 0); 

    } else {
        // KD-tree
        for (int i=0; i < cloud->points.size(); i++) {
            std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            tree->insert(tree->root, point, i, 0); 
        }
    }

    std::vector<std::vector<int>> estimatedClusters = euclideanClusterCustom(cloud, tree, clusterTolerance, maxSize, minSize);
  	for(std::vector<int> cluster : estimatedClusters)
  	{
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
        for (int indice: cluster)
            clusterCloud->points.push_back(cloud->points[indice]);

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1; 
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Get extremums of original cloud
    PointT _minPoint, _maxPoint;
    pcl::getMinMax3D(*cluster, _minPoint, _maxPoint);
    float origHeight = _maxPoint.z - _minPoint.z;

    // Create copy of original cloud with z = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterFilt (new pcl::PointCloud<pcl::PointXYZ>);
    for (PointT point : cluster->points) {
        clusterFilt->points.push_back({point.x, point.y, 0.0f});
    }

    //////////////////////////
    // http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html 
    // https://www.youtube.com/watch?v=mHVwd8gYLnI
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clusterFilt, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clusterFilt, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*clusterFilt, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);

    //////////////////////////

    BoxQ box;
    // Replace Z translation with z coords from original cloud
    bboxTransform[2] = 0.5f * (_maxPoint.z + _minPoint.z);
    box.bboxTransform = bboxTransform;
	box.bboxQuaternion = bboxQuaternion;

	box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    if (box.cube_length == 0) {
        box.cube_length = origHeight;
    } else if (box.cube_width == 0) {
        box.cube_width = origHeight;
    } else {
        box.cube_height = origHeight;
    }

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}