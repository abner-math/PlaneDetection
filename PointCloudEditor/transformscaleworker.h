#ifndef TRANSFORMSCALEWORKER_H
#define TRANSFORMSCALEWORKER_H

#include "worker.h"
#include "simplifiedpointcloud.h"
#include "selectfilter.h"

class TransformScaleWorker : public Worker
{
public:
    TransformScaleWorker(const SelectFilter *filter);

    void pointCloud(PointCloud3d *pointCloud, SimplifiedPointCloud *simplifiedPointCloud)
    {
        mPointCloud = pointCloud;
        mSimplifiedPointCloud = simplifiedPointCloud;
    }

    void scale(float dx, float dy, float dz)
    {
        mScale = Eigen::Vector3f(dx, dy, dz);
    }

    Eigen::Vector3f scale(const Eigen::Vector3f &v);

private:
    const SelectFilter *mFilter;
    PointCloud3d *mPointCloud;
    SimplifiedPointCloud *mSimplifiedPointCloud;
    Eigen::Vector3f mScale;

    void actions() override;

};

#endif // TRANSFORMSCALEWORKER_H
