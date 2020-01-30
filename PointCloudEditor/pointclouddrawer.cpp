#include "pointclouddrawer.h"

#include "colorutils.h"
#include "drawutils.h"
#include "angleutils.h"

#include <iostream>

PointCloudDrawer::PointCloudDrawer(const SelectFilter *filter, const PointCloud3d *pointCloud, const SimplifiedPointCloud *simplifiedPointCloud,
                                   SceneObject::Priority priority)
    : SceneObject(priority)
    , mFilter(filter)
    , mPointCloud(pointCloud)
    , mSimplifiedPointCloud(simplifiedPointCloud)
    , mColorMode(DEFAULT)
    , mDefaultColor(Eigen::Vector3f::Constant(1.0f))
    , mComponent(std::numeric_limits<size_t>::max())
    , mDisplayGeometry(true)
    , mPointSize(1.0f)
{

}

void PointCloudDrawer::pointCloud(const PointCloud3d *pointCloud, const SimplifiedPointCloud *simplifiedPointCloud)
{
    mPointCloud = pointCloud;
    mSimplifiedPointCloud = simplifiedPointCloud;
    mComponent = std::numeric_limits<size_t>::max();
    mVisible = std::vector<bool>(mSimplifiedPointCloud->size(), true);
}

void PointCloudDrawer::color(const Point3d *point, const Eigen::Vector3f &color)
{
    mColors[point] = color;
}

void PointCloudDrawer::clearColor()
{
    mColors.clear();
}

bool PointCloudDrawer::isValid(size_t point) const
{
    if (mComponent != std::numeric_limits<size_t>::max() && mPointCloud->hasConnectivity() &&
            mPointCloud->connectivity()->groupOf(point) != mComponent)
    {
        return false;
    }
    return true;
}

void PointCloudDrawer::getPointPrimitives(std::vector<const Primitive<3>*> &pointPrimitives)
{
    for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
    {
        const Plane *plane = mPointCloud->geometry()->plane(i);
        for (const size_t &inlier : plane->inliers())
        {
            size_t index = mSimplifiedPointCloud->real2virtual(inlier);
            pointPrimitives[index] = plane;
        }
    }

    for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
    {
        const Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
        for (const size_t &inlier : cylinder->inliers())
        {
            size_t index = mSimplifiedPointCloud->real2virtual(inlier);
            pointPrimitives[index] = cylinder;
        }
    }

    for (size_t i = 0; i < mPointCloud->geometry()->numConnections(); i++)
    {
        const Connection *connection = mPointCloud->geometry()->connection(i);
        for (const size_t &inlier : connection->inliers())
        {
            size_t index = mSimplifiedPointCloud->real2virtual(inlier);
            pointPrimitives[index] = connection->extremities()[0].cylinder();
        }
    }
}

void PointCloudDrawer::getMinMaxIntensityCurvature(float &minIntensity, float &maxIntensity,
                                                   float &minCurvature, float &maxCurvature)
{
    minIntensity = minCurvature = std::numeric_limits<float>::max();
    maxIntensity = maxCurvature = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < mSimplifiedPointCloud->size(); i++)
    {
        if (!isValid(i)) continue;
        const Point3d &point = mSimplifiedPointCloud->at(i);
        minIntensity = std::min(minIntensity, point.intensity());
        maxIntensity = std::max(maxIntensity, point.intensity());
        minCurvature = std::min(minCurvature, point.curvature());
        maxCurvature = std::max(maxCurvature, point.curvature());
    }
}

void PointCloudDrawer::drawSupportPlanes()
{
    float extension = mPointCloud->extension().maxSize() * 10;
    Eigen::Vector3f planeU = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f planeV = Eigen::Vector3f(0, 0, -1);
    buffer().color(0.3f, 0.3f, 0.3f);
    Eigen::Vector3f center = mPointCloud->extension().center();
    center.y() = mPointCloud->extension().bottomLeft().y();
    Plane plane(center, Eigen::Vector3f(0, 1, 0), planeU * extension, planeV * extension);
    DrawUtils::outlinePlane(buffer(), plane);
}

void PointCloudDrawer::create()
{
    if (mPointCloud == NULL || mSimplifiedPointCloud == NULL) return;

    std::vector<const Primitive<3>*> pointPrimitives(mSimplifiedPointCloud->size(), NULL);
    getPointPrimitives(pointPrimitives);

    float minIntensity, maxIntensity, minCurvature, maxCurvature;
    getMinMaxIntensityCurvature(minIntensity, maxIntensity, minCurvature, maxCurvature);

    std::map<size_t, Eigen::Vector3f> groupColors;
    if (mPointCloud->hasConnectivity())
    {
        groupColors[0] = Eigen::Vector3f::Constant(1.0f);
        for (const size_t &group : mPointCloud->connectivity()->groups())
        {
            groupColors[group] = ColorUtils::colorWheel(rand() / static_cast<float>(RAND_MAX) * 360.0f);
        }
    }

    bool hasSelection = mFilter->isAnySelected();

    for (size_t i = 0; i < mSimplifiedPointCloud->size(); i++)
    {
        mVisible[i] = false;
        if (pointPrimitives[i] != NULL && mDisplayGeometry) continue;
        const Point3d &point = mSimplifiedPointCloud->at(i);
        Eigen::Vector3f normal = point.normal();
        Eigen::Vector3f color;
        if (mFilter->mode() == SelectFilter::SelectMode::POINT && hasSelection)
        {
            bool selected = false;
            for (const size_t &index : mSimplifiedPointCloud->virtual2real(i))
            {
                if (mFilter->isSelected(index))
                {
                    selected = true;
                    break;
                }
            }
            if (selected)
            {
                color = Eigen::Vector3f(0, 1, 0);
            }
            else
            {
                color = Eigen::Vector3f::Constant(0.3f);
            }
        }
        else
        {
            auto color_it = mColors.find(&point);
            if (color_it != mColors.end())
            {
                color = color_it->second;
            }
            else
            {
                switch (mColorMode)
                {
                case INTENSITIES:
                    if (mPointCloud->hasMode(PointCloud3d::Mode::INTENSITY))
                    {
                        color = ColorUtils::heatMap(point.intensity(), minIntensity, maxIntensity);
                    }
                    else
                    {
                        color = Eigen::Vector3f::Constant(1.0f);
                    }
                    break;
                case CURVATURES:
                    if (mPointCloud->hasMode(PointCloud3d::Mode::CURVATURE))
                    {
                        color = ColorUtils::heatMap(point.curvature(), minCurvature, maxCurvature);
                    }
                    else
                    {
                        color = Eigen::Vector3f::Constant(1.0f);
                    }
                    break;
                case CONNECTED_COMPONENTS:
                    if (mPointCloud->hasConnectivity())
                    {
                        color = groupColors.at(mPointCloud->connectivity()->groupOf(i));
                    }
                    else
                    {
                        color = Eigen::Vector3f::Constant(1.0f);
                    }
                    break;
                default:
                    if (pointPrimitives[i] != NULL)
                    {
                        color = pointPrimitives[i]->color();
                    }
                    else if (mPointCloud->hasMode(PointCloud3d::Mode::COLOR))
                    {
                        color = point.color();
                    }
                    else
                    {
                        color = mDefaultColor;
                    }
                    break;
                }
            }
        }
        buffer().color(color);
        buffer().normal(normal);
        buffer().pointSize(mPointSize);
        buffer().point(point.position());
        mVisible[i] = true;
    }

    for (Connection &connection : mConnections)
    {
        buffer().color(0, 1, 0);
        DrawUtils::outlineRect(buffer(), connection.volume());
    }

    drawSupportPlanes();
}
