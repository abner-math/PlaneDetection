#ifndef CYLINDERSDRAWER_H
#define CYLINDERSDRAWER_H

#include <Eigen/Core>

#include "sceneobject.h"
#include "pointcloud.h"
#include "selectfilter.h"

class CylinderDrawer : public SceneObject
{
public:
    CylinderDrawer(const SelectFilter *filter, const PointCloud3d *pointCloud);

    void pointCloud(const PointCloud3d *pointCloud)
    {
        mPointCloud = pointCloud;
    }

    void displayGeometry(bool display)
    {
        mDisplayGeometry = display;
    }

private:
    const SelectFilter *mFilter;
    const PointCloud3d *mPointCloud;
    bool mDisplayGeometry;

    void create() override;

};

#endif // CYLINDERSDRAWER_H
