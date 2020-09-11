#include "planarpatch.h"
#include "pcacalculator.h"

#include <iostream>

PlanarPatch::PlanarPatch(const PointCloud3d *pointCloud, StatisticsUtils *statistics,
                         const std::vector<size_t> &points, float minAllowedNormal,
                         float maxAllowedDist, float outlierRatio)
    : mPointCloud(pointCloud)
    , mStatistics(statistics)
    , mPoints(points)
    , mOriginalSize(getSize())
    , mNumNewPoints(0)
    , mNumUpdates(0)
    , mStable(false)
    , mMinAllowedNormal(minAllowedNormal)
    , mMaxAllowedDist(maxAllowedDist)
    , mOutlierRatio(outlierRatio)
    , mUsedVisited2(false)
{

}

float PlanarPatch::getSize() const
{
    Eigen::Vector3f min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max = -min;
    for (const size_t &point : mPoints)
    {
        Eigen::Vector3f position = mPointCloud->at(point).position();
        for (size_t i = 0; i < 3; i++)
        {
            min(i) = std::min(min(i), position(i));
            max(i) = std::max(max(i), position(i));
        }
    }
    Rect3d rect(min, max);
    return rect.maxSize();
}

Plane PlanarPatch::getPlane()
{
    mStatistics->size(mPoints.size());
    Eigen::Vector3f center;
    for (size_t dim = 0; dim < 3; dim++)
    {
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            mStatistics->dataBuffer()[i] = mPointCloud->at(mPoints[i]).position()(dim);
        }
        center(dim) = mStatistics->getMedian();
    }
    Eigen::Vector3f normal;
    for (size_t dim = 0; dim < 3; dim++)
    {
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            mStatistics->dataBuffer()[i] = mPointCloud->at(mPoints[i]).normal()(dim);
        }
        normal(dim) = mStatistics->getMedian();
    }
    normal = normal.normalized();
    return Plane(center, normal);
}

float PlanarPatch::getMaxPlaneDist()
{
    mStatistics->size(mPoints.size());
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        const Eigen::Vector3f &position = mPointCloud->at(mPoints[i]).position();
        mStatistics->dataBuffer()[i] = std::abs(mPlane.getSignedDistanceFromSurface(position));
    }
    float minDist, maxDist;
    mStatistics->getMinMaxRScore(minDist, maxDist, 3);
    return maxDist;
}

float PlanarPatch::getMinNormalDiff()
{
    mStatistics->size(mPoints.size());
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        const Eigen::Vector3f &normal = mPointCloud->at(mPoints[i]).normal();
        mStatistics->dataBuffer()[i] = std::abs(normal.dot(mPlane.normal()));
    }
    float minDiff, maxDiff;
    mStatistics->getMinMaxRScore(minDiff, maxDiff, 3);
    return minDiff;
}

bool PlanarPatch::isNormalValid() const
{
    return mMinNormalDiff > mMinAllowedNormal;
}

bool PlanarPatch::isDistValid() const
{
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(mPlane.normal(), basisU, basisV);
    Eigen::Vector3f extreme = basisU * mOriginalSize + mPlane.normal() * mMaxDistPlane;
    return std::abs(extreme.normalized().dot(mPlane.normal())) < mMaxAllowedDist;
}

bool PlanarPatch::isPlanar()
{
    mPlane = getPlane();

    mMinNormalDiff = getMinNormalDiff();
    if (!isNormalValid()) return false;
    size_t countOutliers = 0;
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        bool outlier = mStatistics->dataBuffer()[i] < mMinNormalDiff;
        mOutliers[mPoints[i]] = outlier;
        countOutliers += outlier;
    }
    if (countOutliers > mPoints.size() * mOutlierRatio) return false;

    mMaxDistPlane = getMaxPlaneDist();
    if (!isDistValid()) return false;
    countOutliers = 0;
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        bool outlier = mOutliers[mPoints[i]] || mStatistics->dataBuffer()[i] > mMaxDistPlane;
        mOutliers[mPoints[i]] = outlier;
        countOutliers += outlier;
    }
    if (countOutliers < mPoints.size() * mOutlierRatio)
    {
        removeOutliers();
        return true;
    }
    return false;
}

bool PlanarPatch::isPlanar2()
{
    mMinNormalDiff = 1;
    mMaxDistPlane = 0;
    for (const size_t & point : mPoints)
    {
        float normalDiff = std::abs(mPlane.normal().dot(mPointCloud->at(point).normal()));
        mMinNormalDiff = std::min(mMinNormalDiff, normalDiff);
        float dist = std::abs(mPlane.getSignedDistanceFromSurface(mPointCloud->at(point).position()));
        mMaxDistPlane = std::max(mMaxDistPlane, dist);
    }
    //mMinNormalDiff = getMinNormalDiff();
    //if (!isNormalValid()) return false;
    //mMaxDistPlane = getMaxPlaneDist();
    return isNormalValid() && isDistValid();
}

void PlanarPatch::update()
{
    mPlane = getPlane();
    mMaxDistPlane = getMaxPlaneDist();
    mMinNormalDiff = getMinNormalDiff();
    mVisited.clear();
    if (mNumUpdates > 1)
    {
        mUsedVisited2 = true;
    }
    if (mUsedVisited2) {
        if (mVisited2.empty()) {
            mVisited2 = std::vector<bool>(mPointCloud->size(), false);
        } else {
            std::fill(mVisited2.begin(), mVisited2.end(), false);
        }
    }
    mNumNewPoints = 0;
    ++mNumUpdates;
}

void PlanarPatch::updatePlane()
{
    mPlane = getPlane();
    mVisited.clear();
    if (mNumUpdates > 1)
    {
        mUsedVisited2 = true;
    }
    if (mUsedVisited2) {
        if (mVisited2.empty()) {
            mVisited2 = std::vector<bool>(mPointCloud->size(), false);
        } else {
            std::fill(mVisited2.begin(), mVisited2.end(), false);
        }
    }
    mNumNewPoints = 0;
    ++mNumUpdates;
}

void PlanarPatch::removeOutliers()
{
    mPoints.erase(std::remove_if(mPoints.begin(), mPoints.end(), [this](const size_t &point) {
        return mOutliers[point];
    }), mPoints.end());
    mOutliers.clear();
}
