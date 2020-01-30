#include "normaldrawer.h"

#include "colorutils.h"
#include "geometryutils.h"

NormalDrawer::NormalDrawer(const SimplifiedPointCloud *pointCloud, SceneObject::Priority priority)
    : SceneObject(priority)
    , mPointCloud(pointCloud)
    , mComponent(std::numeric_limits<size_t>::max())
{

}

bool NormalDrawer::isValid(size_t point) const
{
    if (mComponent != std::numeric_limits<size_t>::max() && mPointCloud->hasConnectivity() &&
            mPointCloud->connectivity()->groupOf(point) != mComponent)
    {
        return false;
    }
    return true;
}

void NormalDrawer::create()
{
    if (mPointCloud == NULL || !mPointCloud->hasMode(PointCloud3d::Mode::NORMAL)) return;

    float scaleFactor = mPointCloud->extension().maxSize() * 0.01f;

    float minConfidence = std::numeric_limits<float>::max();
    float maxConfidence = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        if (!isValid(i)) continue;
        const Point3d &point = mPointCloud->at(i);
        minConfidence = std::min(minConfidence, point.normalConfidence());
        maxConfidence = std::max(maxConfidence, point.normalConfidence());
    }

    for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        if (!isValid(i)) continue;
        const Point3d &point = mPointCloud->at(i);
        Eigen::Vector3f position = point.position();
        Eigen::Vector3f normal = point.normal();
        Eigen::Vector3f color = ColorUtils::heatMap(point.normalConfidence(), minConfidence, maxConfidence);
        buffer().color(color);
        buffer().line(position, position + normal * scaleFactor);
    }
}
