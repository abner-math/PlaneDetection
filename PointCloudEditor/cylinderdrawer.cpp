#include "cylinderdrawer.h"

#include "drawutils.h"
#include "colorutils.h"

CylinderDrawer::CylinderDrawer(const SelectFilter *filter, const PointCloud3d *pointCloud)
    : mFilter(filter)
    , mPointCloud(pointCloud)
    , mDisplayGeometry(true)
{

}

void CylinderDrawer::create()
{
    if (mPointCloud == NULL || !mDisplayGeometry) return;

    for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
    {
        const Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
        Eigen::Vector3f color;
        if (mFilter->mode() == SelectFilter::SelectMode::CYLINDER && mFilter->isAnySelected())
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
            if (cylinder->color() == Eigen::Vector3f::Zero())
            {
                color = ColorUtils::colorWheel(rand() % 360);
            }
            else
            {
                color = cylinder->color();
            }
        }
        buffer().color(color);
        DrawUtils::outlineCylinder(buffer(), *cylinder);
    }
}
