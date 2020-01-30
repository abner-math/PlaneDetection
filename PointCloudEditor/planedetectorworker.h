#ifndef PLANEDETECTORWORKER_H
#define PLANEDETECTORWORKER_H

#include "worker.h"
#include "planedetector.h"

class PlaneDetectorWorker : public Worker
{
public:
    enum Mode
    {
        AUTODETECT = 0,
        EXPAND_REGION = 1,
        DETECT_REGION = 2
    };

    PlaneDetectorWorker(const PointCloud3d *pointCloud);

    ~PlaneDetectorWorker();

    void pointCloud(const PointCloud3d *pointCloud);

    void mode(Mode mode)
    {
        mMode = mode;
    }

    Mode mode() const
    {
        return mMode;
    }

    void detectorParams(float minNormal, float maxDist, float outlierRatio)
    {
        mDetector->minNormalDiff(minNormal);
        mDetector->maxDist(maxDist);
        mDetector->outlierRatio(outlierRatio);
    }

    std::vector<Plane*> planes() const
    {
        return mPlanes;
    }

    void region(const std::vector<size_t> &region)
    {
        mRegion = region;
    }

    std::vector<size_t> region() const
    {
        return mRegion;
    }

    PlaneDetector* detector()
    {
        return mDetector;
    }

private:
    PlaneDetector *mDetector;
    Mode mMode;
    std::vector<Plane*> mPlanes;
    std::vector<size_t> mRegion;

    void actions() override;

};

#endif // PLANEDETECTORWORKER_H
