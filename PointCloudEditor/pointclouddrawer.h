#ifndef POINTCLOUDDRAWER_H
#define POINTCLOUDDRAWER_H

#include "sceneobject.h"
#include "pointcloud.h"
#include "simplifiedpointcloud.h"
#include "circle.h"
#include "plane.h"
#include "connection.h"
#include "selectfilter.h"

class PointCloudDrawer : public SceneObject
{
public:
    enum ColorMode
    {
        DEFAULT = 0,
        INTENSITIES = 1,
        CURVATURES = 2,
        CONNECTED_COMPONENTS = 3
    };

    PointCloudDrawer(const SelectFilter *filter, const PointCloud3d *pointCloud, const SimplifiedPointCloud *simplifiedPointCloud,
                     SceneObject::Priority priority = SceneObject::Priority::LOW);

    const PointCloud3d* pointCloud() const
    {
        return mPointCloud;
    }

    void pointCloud(const PointCloud3d *pointCloud, const SimplifiedPointCloud *simplifiedPointCloud);

    void colorMode(ColorMode mode)
    {
        mColorMode = mode;
    }

    void color(const Point3d *point, const Eigen::Vector3f &color);

    void defaultColor(const Eigen::Vector3f &color)
    {
        mDefaultColor = color;
    }

    const Eigen::Vector3f& defaultColor() const
    {
        return mDefaultColor;
    }

    void clearColor();

    void pointSize(float size)
    {
        mPointSize = size;
    }

    float pointSize() const
    {
        return mPointSize;
    }

    void setComponent(size_t component)
    {
        mComponent = component;
    }

    void setConnections(std::vector<Connection> &connections)
    {
        mConnections = connections;
    }

    void displayGeometry(bool display)
    {
        mDisplayGeometry = display;
    }

    bool visible(size_t point) const
    {
        return mVisible[point];
    }

private:
    const SelectFilter *mFilter;
    const PointCloud3d *mPointCloud;
    const SimplifiedPointCloud *mSimplifiedPointCloud;
    ColorMode mColorMode;
    std::map<const Point3d*, Eigen::Vector3f> mColors;
    Eigen::Vector3f mDefaultColor;
    size_t mComponent;
    bool mDisplayGeometry;
    float mPointSize;
    std::vector<Connection> mConnections;
    std::vector<bool> mVisible;

    void getMinMaxIntensityCurvature(float &minIntensity, float &maxIntensity, float &minCurvature, float &maxCurvature);

    void getPointPrimitives(std::vector<const Primitive<3>*> &pointPrimitives);

    bool isValid(size_t point) const;

    void drawSupportPlanes();

    void create() override;

};

#endif // POINTCLOUDDRAWER_H
