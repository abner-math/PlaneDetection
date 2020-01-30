#include "planedrawer.h"

#include "drawutils.h"
#include "colorutils.h"

PlaneDrawer::PlaneDrawer(const SelectFilter *filter, const PointCloud3d *pointCloud)
    : mFilter(filter)
    , mPointCloud(pointCloud)
    , mDisplayGeometry(true)
{

}

void PlaneDrawer::create()
{
    if (mPointCloud == NULL || !mDisplayGeometry) return;

    for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
    {
        const Plane *plane = mPointCloud->geometry()->plane(i);
        Eigen::Vector3f color;
        if (mFilter->mode() == SelectFilter::SelectMode::PLANE && mFilter->isAnySelected())
        {
            if (mFilter->isSelected(i))
            {
                color = Eigen::Vector3f(0, 1, 0);
            }
            else
            {
                color = Eigen::Vector3f::Constant(0.3f);
            }
        }
        else if (mFilter->isAnySelected())
        {
            color = Eigen::Vector3f::Constant(0.3f);
        }
        else
        {
            if (plane->color() == Eigen::Vector3f::Zero())
            {
                color = ColorUtils::colorWheel(rand() % 360);
            }
            else
            {
                color = plane->color();
            }
        }

        buffer().color(color);
        buffer().normal(plane->normal());
        DrawUtils::outlinePlane(buffer(), *plane);

        Eigen::Vector3f min, max;
        min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
        max = -min;

        for (const size_t &inlier : plane->inliers())
        {
            Eigen::Vector3f position = mPointCloud->at(inlier).position();
            for (size_t i = 0; i < 3; i++)
            {
                min(i) = std::min(min(i), position(i));
                max(i) = std::max(max(i), position(i));
            }
        }

        Rect3d r(min, max);
        //DrawUtils::outlineRect(buffer(), r);
    }
}
