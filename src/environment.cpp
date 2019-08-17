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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_points_clouds = lidar->scan();
    //renderRays(viewer, lidar->position, input_points_clouds);
    //renderPointCloud(viewer, input_points_clouds, "pcl", Color(6,9,0));
    ProcessPointClouds<pcl::PointXYZ> * procPCL = new ProcessPointClouds<pcl::PointXYZ>();
    
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(input_points_clouds, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("/home/lunarpulse/Documents/SensorFusion/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, .2,
    Eigen::Vector4f(-20.0, -6.0, -1.0, 1.0), Eigen::Vector4f(20.0, 6.0, 2.0, 1.0));
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    auto clusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 40, 25000);
    uint clusterID = 0;
    std::vector<Color> colours = {Color(0.5,0,0), Color(0., 0.5,0),Color(0.0,0,0.5),
                                Color(0.5,0.5,0),Color(0.,0.5,0.5),Color(0.5,0,0.5)
                                ,Color(1.0,0,0.5),Color(0.5,1.0, 0.0),Color(0.5,0,1.0)
                                ,Color(1.0,0.5,0.5),Color(0.5,1.0, 0.5),Color(0.5,0.5,1.0)
                                ,Color(1.0,0.5,0.5),Color(0.5,1.0, 0.5),Color(0.5,0.5,1.0)
                                ,Color(1.0,1.0,0.5),Color(0.5,1.0, 1.0),Color(1.0,0.5,1.0) };

    for (auto &&cluster : clusters)
    {
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterID), colours[clusterID%colours.size()]);
        renderBox( viewer, pointProcessorI->BoundingBox(cluster), clusterID,
         Color(1 - colours[clusterID%colours.size()].r,
         1 - colours[clusterID%colours.size()].b,
         1- 1 - colours[clusterID%colours.size()].g), 0.25f);
        clusterID++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, .2,
        Eigen::Vector4f(-10.0, -6.0, -1.0, 1.0), Eigen::Vector4f(30.0, 6.0, 2.0, 1.0));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
        pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    auto clusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 40, 25000);
    uint clusterID = 0;
    std::vector<Color> colours = {Color(0.5,0,0), Color(0., 0.5,0),Color(0.0,0,0.5),
                                Color(0.5,0.5,0),Color(0.,0.5,0.5),Color(0.5,0,0.5)
                                ,Color(1.0,0,0.5),Color(0.5,1.0, 0.0),Color(0.5,0,1.0)
                                ,Color(1.0,0.5,0.5),Color(0.5,1.0, 0.5),Color(0.5,0.5,1.0)
                                ,Color(1.0,0.5,0.5),Color(0.5,1.0, 0.5),Color(0.5,0.5,1.0)
                                ,Color(1.0,1.0,0.5),Color(0.5,1.0, 1.0),Color(1.0,0.5,1.0) };

    for (auto &&cluster : clusters)
    {
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterID), colours[clusterID%colours.size()]);
        renderBox( viewer, pointProcessorI->BoundingBox(cluster), clusterID,
            Color(1 - colours[clusterID%colours.size()].r,
            1 - colours[clusterID%colours.size()].b,
            1- 1 - colours[clusterID%colours.size()].g), 0.25f);
        clusterID++;
    }
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/lunarpulse/Documents/SensorFusion/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}