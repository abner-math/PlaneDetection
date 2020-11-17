#ifndef PRIMITIVEDETECTOR_H
#define PRIMITIVEDETECTOR_H

#include <set>
#include <iostream>
#include <numeric>

#include "primitive.h"
#include "pointcloud.h"

template <size_t DIMENSION, class PRIMITIVE_TYPE>
class PrimitiveDetector
{
public:
    PrimitiveDetector(const PointCloud<DIMENSION> *pointCloud)
        : mPointCloud(pointCloud)
        , mNumRemovedPoints(0)
    {
        static_assert(std::is_base_of<Primitive<DIMENSION>, PRIMITIVE_TYPE>::value, "Template type is not derived from Primitive.");
        clearRemovedPoints();
    }

    virtual ~PrimitiveDetector()
    {

    }

    const PointCloud<DIMENSION>* pointCloud() const
    {
        return mPointCloud;
    }

    void pointCloud(const PointCloud<DIMENSION> *pointCloud)
    {
        mPointCloud = pointCloud;
        clearRemovedPoints();
    }

    void removePoint(const size_t &point)
    {
        if (mRemoved[point]) return;
        mRemoved[point] = true;
        ++mNumRemovedPoints;
    }

    void clearRemovedPoints()
    {
        if (mPointCloud == NULL) return;
        mNumRemovedPoints = 0;
        mRemoved = std::vector<bool>(mPointCloud->size(), false);
        mAvailablePoints = std::vector<size_t>(mPointCloud->size());
        std::iota(mAvailablePoints.begin(), mAvailablePoints.end(), 0);
    }

    inline bool isRemoved(const size_t point) const
    {
        return mRemoved[point];
    }

    inline size_t numAvailablePoints() const
    {
        return mPointCloud->size() - mNumRemovedPoints;
    }

    const std::vector<size_t>& availablePoints() const
    {
        return mAvailablePoints;
    }

    void updateAvailablePoints()
    {
        mAvailablePoints.clear();
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (!mRemoved[i])
            {
                mAvailablePoints.push_back(i);
            }
        }
    }

    virtual std::set<PRIMITIVE_TYPE*> detect() = 0;

private:
    const PointCloud<DIMENSION> *mPointCloud;
    std::vector<bool> mRemoved;
    size_t mNumRemovedPoints;
    std::vector<size_t> mAvailablePoints;

};

#endif // PRIMITIVEDETECTOR_H
