#ifndef GAUSSIANMAPDRAWER_H
#define GAUSSIANMAPDRAWER_H

#include "simplifiedpointcloud.h"
#include "sceneobject.h"
#include "plane.h"
#include "boundaryvolumehierarchy.h"

class GaussianMapDrawer : public SceneObject
{
public:
    GaussianMapDrawer(const SimplifiedPointCloud *pointCloud, const Eigen::Vector3f &center, float radius);

    GaussianMapDrawer(const SimplifiedPointCloud *pointCloud, float centerX, float centerY, float centerZ, float radius);

    void pointCloud(SimplifiedPointCloud *pointCloud);

    void setComponent(size_t component)
    {
        mComponent = component;
    }

private:
    const SimplifiedPointCloud *mPointCloud;
    Eigen::Vector3f mCenter;
    float mRadius;
    size_t mComponent;

    void create() override;

    bool isValid(size_t point) const;

};

#endif // GAUSSIANMAPDRAWER_H
