#ifndef TRANSFORMTRANSLATEWORKER_H
#define TRANSFORMTRANSLATEWORKER_H

#include "worker.h"
#include "simplifiedpointcloud.h"
#include "selectfilter.h"

class TransformTranslateWorker : public Worker
{
public:
    TransformTranslateWorker(const SelectFilter *filter);

    void pointCloud(PointCloud3d *pointCloud, SimplifiedPointCloud *simplifiedPointCloud)
    {
        mPointCloud = pointCloud;
        mSimplifiedPointCloud = simplifiedPointCloud;
    }

    void translation(float dx, float dy, float dz)
    {
        mTranslation = Eigen::Vector3f(dx, dy, dz);
    }

    const Eigen::Vector3f& translation() const
    {
        return mTranslation;
    }

private:
    const SelectFilter *mFilter;
    PointCloud3d *mPointCloud;
    SimplifiedPointCloud *mSimplifiedPointCloud;
    Eigen::Vector3f mTranslation;

    void actions() override;

};

#endif // TRANSFORMTRANSLATEWORKER_H
