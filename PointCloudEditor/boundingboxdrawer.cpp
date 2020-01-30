#include "boundingboxdrawer.h"

#include "drawutils.h"

BoundingBoxDrawer::BoundingBoxDrawer(const SimplifiedPointCloud *pointCloud, SceneObject::Priority priority)
    : SceneObject(priority)
    , mPointCloud(pointCloud)
{

}

void BoundingBoxDrawer::create()
{
    if (mPointCloud == NULL) return;

    buffer().color(0, 1, 0);
    DrawUtils::outlineRect(buffer(), mPointCloud->extension());
}
