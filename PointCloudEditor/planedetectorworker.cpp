#include "planedetectorworker.h"

PlaneDetectorWorker::PlaneDetectorWorker(const PointCloud3d *pointCloud)
    : mDetector(new PlaneDetector(pointCloud))
    , mMode(AUTODETECT)
{

}

PlaneDetectorWorker::~PlaneDetectorWorker()
{
    delete mDetector;
}

void PlaneDetectorWorker::pointCloud(const PointCloud3d *pointCloud)
{
    mDetector->pointCloud(pointCloud);
}

void PlaneDetectorWorker::actions()
{
    switch (mMode)
    {
    case Mode::AUTODETECT:
    {
        emit workerStatus(QString("Detecting planes..."));
        std::set<Plane*> planes = mDetector->detect();
        mPlanes = std::vector<Plane*>(planes.begin(), planes.end());
        break;
    }
    case Mode::EXPAND_REGION:
        emit workerStatus(QString("Expanding region..."));
        mDetector->growRegion(mRegion);
        break;
    case Mode::DETECT_REGION:
        emit workerStatus(QString("Detecting plane..."));
        mPlanes.clear();
        mPlanes.push_back(mDetector->detectPlane(mRegion));
        break;
    }
}

