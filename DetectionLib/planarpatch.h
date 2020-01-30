#ifndef PLANEDETECTORPATCH_H
#define PLANEDETECTORPATCH_H

#include <unordered_set>
#include <unordered_map>

#include <Eigen/Core>
#include "plane.h"
#include "pointcloud.h"
#include "statisticsutils.h"

struct RotatedRect
{
    Eigen::Matrix3Xf matrix;
    Eigen::Matrix3f basis;
    float area;
    Rect3d rect;

    RotatedRect()
        : area(std::numeric_limits<float>::max())
    {

    }

    RotatedRect(const Eigen::Matrix3Xf &matrix, const Eigen::Matrix3f &basis, float degrees)
    {
        Eigen::Matrix3f newBasis;
        newBasis.row(0) = AngleUtils::rotate(basis.row(0), degrees, basis.row(2));
        newBasis.row(1) = AngleUtils::rotate(basis.row(1), degrees, basis.row(2));
        newBasis.row(2) = basis.row(2);
        Eigen::Matrix3Xf newMatrix = newBasis * matrix;
        Eigen::Vector3f max = newMatrix.rowwise().maxCoeff();
        Eigen::Vector3f min = newMatrix.rowwise().minCoeff();
        float w = max(0) - min(0);
        float h = max(1) - min(1);
        float area = w * h;
        this->matrix = newMatrix;
        this->area = area;
        this->basis = newBasis;
        this->rect = Rect3d(min, max);
    }

};

class PlanarPatch
{
public:
    PlanarPatch(const PointCloud3d *mPointCloud, StatisticsUtils *statistics,
                const std::vector<size_t> &mPoints, float minAllowedNormal,
                float maxAllowedDist, float outlierRatio);

    size_t index() const
    {
        return mIndex;
    }

    void index(size_t index)
    {
        mIndex = index;
    }

    inline bool isInlier(size_t point) const
    {
        return std::abs(mPlane.normal().dot(mPointCloud->at(point).normal())) > mMinNormalDiff &&
                std::abs(mPlane.getSignedDistanceFromSurface(mPointCloud->at(point).position())) < mMaxDistPlane;
    }

    inline bool isVisited(size_t point) const
    {
        if (!mVisited2.empty()) {
            return mVisited2[point];
        }
        return mVisited.find(point) != mVisited.end();
    }

    inline void visit(size_t point)
    {
        if (!mVisited2.empty()) {
            mVisited2[point] = true;
        } else {
            mVisited.insert(point);
        }
    }

    inline void addPoint(size_t point)
    {
        mPoints.push_back(point);
        ++mNumNewPoints;
        mOutliers[point] = false;
    }

    bool isPlanar();

    bool isPlanar2();

    void update();

    void updatePlane();

    const std::vector<size_t>& points() const
    {
        return mPoints;
    }

    void points(const std::vector<size_t> &points)
    {
        mPoints = points;
    }

    const Plane& plane() const
    {
        return mPlane;
    }

    void plane(const Plane &plane)
    {
        mPlane = plane;
    }

    const Eigen::Vector3f center() const
    {
        return mPlane.center();
    }

    void center(const Eigen::Vector3f &center)
    {
        mPlane.center(center);
    }

    const Eigen::Vector3f normal() const
    {
        return mPlane.normal();
    }

    void normal(const Eigen::Vector3f &normal)
    {
        mPlane.normal(normal);
    }

    float maxDistPlane() const
    {
        return mMaxDistPlane;
    }

    void maxDistPlane(float maxDistPlane)
    {
        mMaxDistPlane = maxDistPlane;
    }

    float minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(float minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    RotatedRect& rect()
    {
        return mRect;
    }

    size_t& numNewPoints()
    {
        return mNumNewPoints;
    }

    bool& stable()
    {
        return mStable;
    }

    size_t numUpdates() const
    {
        return mNumUpdates;
    }

    size_t originalSize() const
    {
        return mOriginalSize;
    }

    void removeOutliers();

    float getSize() const;

private:
    const PointCloud3d *mPointCloud;
    StatisticsUtils *mStatistics;
    std::vector<size_t> mPoints;
    float mOriginalSize;
    size_t mIndex;
    Plane mPlane;
    float mMaxDistPlane;
    float mMinNormalDiff;
    RotatedRect mRect;
    size_t mNumNewPoints;
    size_t mNumUpdates;
    std::unordered_set<size_t> mVisited;
    std::vector<bool> mVisited2;
    std::unordered_map<size_t, bool> mOutliers;
    bool mStable;
    float mMinAllowedNormal;
    float mMaxAllowedDist;
    float mOutlierRatio;
    bool mUsedVisited2;

    bool isNormalValid() const;

    bool isDistValid() const;

    float getMaxPlaneDist();

    float getMinNormalDiff();

    Plane getPlane();

};

#endif // PLANEDETECTORPATCH_H
