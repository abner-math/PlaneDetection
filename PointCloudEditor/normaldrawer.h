#ifndef NORMALDRAWER_H
#define NORMALDRAWER_H

#include "simplifiedpointcloud.h"
#include "sceneobject.h"
#include "plane.h"

class NormalDrawer : public SceneObject
{
public:
    NormalDrawer(const SimplifiedPointCloud *pointCloud, SceneObject::Priority priority = SceneObject::Priority::LOW);

    void pointCloud(const SimplifiedPointCloud *pointCloud)
    {
        mPointCloud = pointCloud;
        mComponent = std::numeric_limits<size_t>::max();
    }

    void setComponent(size_t component)
    {
        mComponent = component;
    }

private:
    const SimplifiedPointCloud *mPointCloud;
    size_t mComponent;

    void create() override;

    bool isValid(size_t point) const;

};

#endif // NORMALDRAWER_H
