#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "pointcloud.h"
#include "primitivedetector.h"
#include "planarpatch.h"
#include "boundaryvolumehierarchy.h"

class PlaneDetector : public PrimitiveDetector<3, Plane>
{
public:
    PlaneDetector(const PointCloud3d *pointCloud);

    Plane* detectPlane(const std::vector<size_t> &points);

    void growRegion(std::vector<size_t> &points);

    void delimitPlane(PlanarPatch *patch);

    void delimitPlane(Plane *plane);

    float minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(float minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    float maxDist() const
    {
        return mMaxDist;
    }

    void maxDist(float maxDist)
    {
        mMaxDist = maxDist;
    }

    float outlierRatio() const
    {
        return mOutlierRatio;
    }

    void outlierRatio(float outlierRatio)
    {
        mOutlierRatio = outlierRatio;
    }

    std::set<Plane*> detect() override;

private:
    std::vector<PlanarPatch*> mPatchPoints;
    float mMinNormalDiff;
    float mMaxDist;
    float mOutlierRatio;

    bool detectPlanarPatches(Octree *node, StatisticsUtils *statistics, size_t minNumPoints, std::vector<PlanarPatch*> &patches);

    void growPatches(std::vector<PlanarPatch*> &patches, bool relaxed = false);

    void mergePatches(std::vector<PlanarPatch*> &patches);

    bool updatePatches(std::vector<PlanarPatch*> &patches);

    void getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier);

    bool isFalsePositive(PlanarPatch *patch);

};

#endif // PLANEDETECTOR_H
