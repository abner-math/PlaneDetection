#ifndef BOUNDINGBOXDRAWER_H
#define BOUNDINGBOXDRAWER_H

#include "simplifiedpointcloud.h"
#include "sceneobject.h"

class BoundingBoxDrawer : public SceneObject
{
public:
    BoundingBoxDrawer(const SimplifiedPointCloud *pointCloud, SceneObject::Priority priority = SceneObject::Priority::LOW);

    const SimplifiedPointCloud* pointCloud() const
    {
        return mPointCloud;
    }

    void pointCloud(SimplifiedPointCloud *pointCloud)
    {
        mPointCloud = pointCloud;
    }

private:
    const SimplifiedPointCloud *mPointCloud;

    void create() override;

};

#endif // BOUNDINGBOXDRAWER_H
