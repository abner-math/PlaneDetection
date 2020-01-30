#include "gaussianmapdrawer.h"

#include <iostream>
#include "colorutils.h"
#include "drawutils.h"

GaussianMapDrawer::GaussianMapDrawer(const SimplifiedPointCloud *pointCloud, const Eigen::Vector3f &center, float radius)
    : mPointCloud(pointCloud)
    , mCenter(center)
    , mRadius(radius)
    , mComponent(std::numeric_limits<size_t>::max())
{

}

GaussianMapDrawer::GaussianMapDrawer(const SimplifiedPointCloud *pointCloud, float centerX, float centerY, float centerZ, float radius)
    : GaussianMapDrawer(pointCloud, Eigen::Vector3f(centerX, centerY, centerZ), radius)
{

}

void GaussianMapDrawer::pointCloud(SimplifiedPointCloud *pointCloud)
{
    mPointCloud = pointCloud;
    mComponent = std::numeric_limits<size_t>::max();
}

bool GaussianMapDrawer::isValid(size_t point) const
{
    if (mComponent != std::numeric_limits<size_t>::max() && mPointCloud->hasConnectivity() &&
            mPointCloud->connectivity()->groupOf(point) != mComponent)
    {
        return false;
    }
    return true;
}

void GaussianMapDrawer::create()
{
    // Axis
    buffer().color(1, 0, 0);
    buffer().line(mCenter, mCenter + Eigen::Vector3f(mRadius, 0, 0));
    buffer().color(0, 1, 0);
    buffer().line(mCenter, mCenter + Eigen::Vector3f(0, mRadius, 0));
    buffer().color(0, 0, 1);
    buffer().line(mCenter, mCenter + Eigen::Vector3f(0, 0, mRadius));

    // Sphere
    buffer().color(0.3f, 0.3f, 0.3f);
    DrawUtils::outlineSphere(buffer(), mCenter, mRadius);

    // Points
    if (mPointCloud == NULL) return;

    float minConfidence = std::numeric_limits<float>::max();
    float maxConfidence = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        if (!isValid(i)) continue;
        float confidence = mPointCloud->at(i).normalConfidence();
        if (confidence < minConfidence) minConfidence = confidence;
        if (confidence > maxConfidence) maxConfidence = confidence;
    }

    for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        if (!isValid(i)) continue;
        const Point3d &point = mPointCloud->at(i);
        Eigen::Vector3f normal = point.normal();
        float confidence = point.normalConfidence();
        buffer().color(ColorUtils::heatMap(confidence, minConfidence, maxConfidence));
        buffer().point(mCenter + normal * mRadius);
    }
}
