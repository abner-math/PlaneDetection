#ifndef TRANSFORMROTATEWORKER_H
#define TRANSFORMROTATEWORKER_H

#include "worker.h"
#include "simplifiedpointcloud.h"
#include "selectfilter.h"

class TransformRotateWorker : public Worker
{
public:
    TransformRotateWorker(const SelectFilter *filter);

    void pointCloud(PointCloud3d *pointCloud, SimplifiedPointCloud *simplifiedPointCloud)
    {
        mPointCloud = pointCloud;
        mSimplifiedPointCloud = simplifiedPointCloud;
    }

    void rotate(float x, float y, float z, float degrees)
    {
        mAxis = Eigen::Vector3f(x, y, z).normalized();
        mDegrees = degrees;
    }

    Eigen::Vector3f rotate(const Eigen::Vector3f &v);

private:
    const SelectFilter *mFilter;
    PointCloud3d *mPointCloud;
    SimplifiedPointCloud *mSimplifiedPointCloud;
    Eigen::Vector3f mAxis;
    float mDegrees;

    void actions() override;

};


#endif // TRANSFORMROTATEWORKER_H
