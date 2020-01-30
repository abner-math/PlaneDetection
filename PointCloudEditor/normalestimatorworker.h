#ifndef NORMALESTIMATORWORKER_H
#define NORMALESTIMATORWORKER_H

#include "normalestimator.h"
#include "worker.h"

class NormalEstimatorWorker : public Worker
{
    Q_OBJECT
public:
    NormalEstimatorWorker(PointCloud3d *pointCloud);

    void pointCloud(PointCloud3d *pointCloud)
    {
        mPointCloud = pointCloud;
    }

    void numNeighbors(size_t numNeighbors)
    {
        mNumNeighbors = numNeighbors;
    }

    void speed(NormalEstimator3d::Speed speed)
    {
        mSpeed = speed;
    }

private:
    PointCloud3d *mPointCloud;
    size_t mNumNeighbors;
    NormalEstimator3d::Speed mSpeed;

    void actions() override;

};

#endif // NORMALESTIMATORWORKER_H
