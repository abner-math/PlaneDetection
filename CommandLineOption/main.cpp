#include <pointcloudio.hpp>
#include <planedetector.h>
#include <normalestimator.h>
#include <boundaryvolumehierarchy.h>
#include <connectivitygraph.h>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: <input_point_cloud (XYZ format)> <output file (.txt)>" << std::endl;
        return -1;
    }
    std::string inputFileName(argv[1]);
    std::string outputFileName(argv[2]);

    std::cout << "Reading the point cloud..." << std::endl;
    PointCloudIO pointCloudIO;
    // many formats are supported from this class, but XYZ as chosen for being popular and simple
    PointCloud3d *pointCloud = pointCloudIO.loadFromXYZ(inputFileName);

    // you can skip the normal estimation if you point cloud already have normals
    std::cout << "Estimating normals..." << std::endl;
    size_t normalsNeighborSize = 30;
    Octree octree(pointCloud);
    octree.partition(10, 30);
    ConnectivityGraph *connectivity = new ConnectivityGraph(pointCloud->size());
    pointCloud->connectivity(connectivity);
    NormalEstimator3d estimator(&octree, normalsNeighborSize, NormalEstimator3d::QUICK);
    std::cout << pointCloud->size() << std::endl;
    for (size_t i = 0; i < pointCloud->size(); i++)
    {
        if (i % 10000 == 0)
        {
            std::cout << i / float(pointCloud->size()) * 100 << "%..." << std::endl;
        }
        NormalEstimator3d::Normal normal = estimator.estimate(i);
        connectivity->addNode(i, normal.neighbors);
        (*pointCloud)[i].normal(normal.normal);
        (*pointCloud)[i].normalConfidence(normal.confidence);
        (*pointCloud)[i].curvature(normal.curvature);
    }
            
    std::cout << "Detecting planes..." << std::endl;
    PlaneDetector detector(pointCloud);
    detector.minNormalDiff(0.5f);
    detector.maxDist(0.258819f);
    detector.outlierRatio(0.75f);

    std::set<Plane*> planes = detector.detect();
    std::cout << planes.size() << std::endl;

    std::cout << "Saving results..." << std::endl;
    Geometry *geometry = pointCloud->geometry();
    for (Plane *plane : planes) 
    {
        geometry->addPlane(plane);
    }
    // many output formats are allowed. if you want to run our 'compare_plane_detector', uncomment the line below and comment the rest
    //pointCloudIO.saveGeometry(geometry, outputFileName);
    std::ofstream outputFile(outputFileName + ".txt");
    for (Plane *plane : planes)
    {
        outputFile << "Normal: " << plane->normal()[0] << ", " << plane->normal()[1] << ", " << plane->normal()[2] <<
                    "; Center: " << plane->center()[0] << ", " << plane->center()[1] << ", " << plane->center()[2] << std::endl;
    }

    delete pointCloud;
    return 0;
}