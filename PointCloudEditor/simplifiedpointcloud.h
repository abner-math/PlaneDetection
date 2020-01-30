#ifndef SIMPLIFIEDPOINTCLOUD_H
#define SIMPLIFIEDPOINTCLOUD_H

#include <set>
#include <memory>
#include <map>

#include "boundaryvolumehierarchy.h"

class SimplifiedPointCloud : public PointCloud3d
{
public:
    SimplifiedPointCloud(const PointCloud3d *pointCloud, size_t levels = 10, size_t minNumPoints = 1);

    PointCloud3d* pointCloud() const;

    bool hasConnectivity() const
    {
        return mPointCloud->hasConnectivity();
    }

    const ConnectivityGraph* connectivity() const
    {
        return mPointCloud->connectivity();
    }

    size_t real2virtual(size_t real) const
    {
        return mReal2Virtual.at(real);
    }

    const std::vector<size_t>& virtual2real(size_t virt) const
    {
        return mVirtual2Real[virt];
    }

    size_t levels() const
    {
        return mLevels;
    }

    void levels(size_t levels)
    {
        mLevels = levels;
    }

    void update();

private:
    const PointCloud3d *mPointCloud;
    size_t mLevels;
    size_t mMinNumPoints;
    std::map<size_t, size_t> mReal2Virtual;
    std::vector<std::vector<size_t> > mVirtual2Real;
    std::vector<size_t> mGroupIndices;
    std::vector<bool> mSelected;

    Point3d getAveragePoint(const Octree *node) const;

    void addPoints(const Octree *node);

};

#endif // SIMPLIFIEDPOINTCLOUD_H
