/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);  // on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr InCloud = lidar->scan();
    // renderRays(viewer, lidar->position, InCloud);
    // renderPointCloud(viewer, InCloud, "Input cloud", Color(1,1,1));

    // Create point processor (cloud segmentating)
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(InCloud, 100, 0.2);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Point processor for clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // Box box = pointProcessor->BoundingBox(cluster);
        BoxQ box = pointProcessor->BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
  
}

void cityBlockFrame(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud,"inputCloud");

    // Experiment with the ? values and find what works best
    float filterRes = 0.2; // default = 0.01, m.
    float minDistance = 3.0; // to filter points from car's roof, m.; works only on `manual filtering mode`
    Eigen::Vector4f minPoint(-20, -6, -3, 1); 
    Eigen::Vector4f maxPoint(20, 6, 5, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZI> ());
    filterCloud = pointProcessorI->FilterCloud(inputCloud, minDistance, filterRes, minPoint, maxPoint);

    renderPointCloud(viewer, filterCloud, "filterCloud");

    // Cloud segmentating
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Point processor for clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.7, 8, 500);

    int clusterId = 0, colorId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (colorId > colors.size())
            colorId = 0;

        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), colors[colorId]);

        Box box = pointProcessor->BoundingBox(cluster); // boxes without rotation
        // BoxQ box = pointProcessor->BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
        ++colorId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {

    // Experiment with the ? values and find what works best
    float filterRes = 0.2; // default = 0.01, m.
    float minDistance = 1.5; // to filter points from car's roof, m.; works only on `manual filtering mode`
    Eigen::Vector4f minPoint(-20, -6, -2, 1); 
    Eigen::Vector4f maxPoint(20, 6, 5, 1); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZI> ());
    filterCloud = pointProcessorI->FilterCloud(inputCloud, minDistance, filterRes, minPoint, maxPoint);

    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // Cloud segmentation
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 30, 0.25);

    // maxIters could be `-1` - then the number of iterations estimated automatically
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlaneCustom(filterCloud, 40, 0.25, true);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::cout << "Plane points: " << segmentCloud.second->points.size() << ", out of total: " << filterCloud->points.size() << std::endl;

    // Point processor for clustering
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.7, 10, 500);

    // the last argument turns on/off median balancing of kd-tree
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->ClusteringCustom(segmentCloud.first, 0.9, 20, 500, false); // 0.5; 10; 140

    int clusterId = 0, colorId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(0,0,1), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (colorId > colors.size())
            colorId = 0;

        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[colorId]);

        Box box = pointProcessor->BoundingBox(cluster); // boxes without rotation
        // BoxQ box = pointProcessor->BoundingBoxQ(cluster);

        if (
            ((box.cube_length > 0.8) && (box.cube_width > 0.8) && (box.cube_height > 0.4)) && 
            ((box.cube_length < 6.0) && (box.cube_width < 6.0) && (box.cube_height < 5.0)) 
           ) 
        {
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
        ++colorId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // View single frame
    // simpleHighway(viewer); // from silmulator
    // cityBlockFrame(viewer); // real PCD

    // PC stream
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // For stream processing
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce(100); // argument is a wait time in ms.
    } 
}