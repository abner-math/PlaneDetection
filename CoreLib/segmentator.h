#ifndef SEGMENTATOR_H
#define SEGMENTATOR_H

#include <queue>
#include <iostream>

#include "pointcloud.h"
#include "angleutils.h"

#define MIN_NUM_POINTS 100

// reference article: Robust segmentation in laser scanning 3D point cloud data.
template <size_t DIMENSION>
class Segmentator
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    Segmentator(const PointCloud<DIMENSION> *pointCloud)
        : mPointCloud(pointCloud)
        , mNumVisitedPoints(0)
    {
        if (!pointCloud->hasConnectivity())
        {
            throw "You need to estimate normals first!";
        }
        mNewGraph = new ConnectivityGraph(mPointCloud->size());
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            mNewGraph->addNode(i, mPointCloud->connectivity()->neighbors(i));
        }
        mVisitedPoints = std::vector<bool>(pointCloud->size(), false);
    }

    Segmentator(const PointCloud<DIMENSION> *pointCloud, ConnectivityGraph *graph)
        : mPointCloud(pointCloud)
        , mNumVisitedPoints(0)
    {
        if (!pointCloud->hasConnectivity())
        {
            throw "You need to estimate normals first!";
        }
        mNewGraph = graph;
        mVisitedPoints = std::vector<bool>(pointCloud->size(), false);
    }

    void segmentNewGroup()
    {
        if (mNumVisitedPoints / (float)mPointCloud->size() > 0.96f)
        {
            mNumVisitedPoints = mPointCloud->size();
            return;
        }
        size_t minCurvature = getMinCurvaturePoint();
        if (minCurvature == std::numeric_limits<size_t>::max()) return;
        std::queue<size_t> queue;
        queue.push(minCurvature);
        mVisitedPoints[minCurvature] = true;
        std::vector<size_t> pointsOnRegion;
        while (!queue.empty())
        {
            size_t seedPoint = queue.front();
            queue.pop();
            ++mNumVisitedPoints;
            pointsOnRegion.push_back(seedPoint);
            for (const size_t &neighbor : mNewGraph->neighbors(seedPoint))
            {
                if (!mVisitedPoints[neighbor] && getAngleBetween(seedPoint, neighbor) > sAngleThreshold)
                {
                    mVisitedPoints[neighbor] = true;
                    queue.push(neighbor);
                }
            }
        }
        size_t group = mNewGraph->numGroups() + 1;
        mNewGraph->addGroup(group, pointsOnRegion);
    }

    void segmentGroupsBFS()
    {
        std::queue<size_t> queue;
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (!mVisitedPoints[i])
            {
                mVisitedPoints[i] = true;
                ++mNumVisitedPoints;
                queue.push(i);
                std::vector<size_t> points;
                while (!queue.empty())
                {
                    size_t point = queue.front();
                    queue.pop();
                    points.push_back(point);
                    for (const size_t &neighbor : mNewGraph->neighbors(point))
                    {
                        if (!mVisitedPoints[neighbor])
                        {
                            mVisitedPoints[neighbor] = true;
                            queue.push(neighbor);
                        }
                    }
                }
                size_t group = mNewGraph->numGroups() + 1;
                mNewGraph->addGroup(group, points);
            }
        }
    }

    void removeInvalidGroups()
    {
        std::set<size_t> groups = mNewGraph->groups();
        for (const size_t &group : groups)
        {
            if (mNewGraph->numPointsInGroup(group) < MIN_NUM_POINTS)
            {
                mNewGraph->removeGroup(group);
            }
        }
    }

    size_t numVisitedPoints()
    {
        return mNumVisitedPoints;
    }

    ConnectivityGraph* getNewConnectivityGraph()
    {
        return mNewGraph;
    }

private:
    const static float sAngleThreshold;
    const PointCloud<DIMENSION> *mPointCloud;
    ConnectivityGraph *mNewGraph;
    std::vector<bool> mVisitedPoints;
    size_t mNumVisitedPoints;

    size_t getMinCurvaturePoint()
    {
        size_t minCurvature = std::numeric_limits<size_t>::max();
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (!mVisitedPoints[i] && (minCurvature == std::numeric_limits<size_t>::max() ||
                                       mPointCloud->at(i).curvature() < mPointCloud->at(minCurvature).curvature()))
            {
                minCurvature = i;
            }
        }
        return minCurvature;
    }

    inline float getAngleBetween(size_t pointA, size_t pointB)
    {
        return std::abs(mPointCloud->at(pointA).normal().dot(mPointCloud->at(pointB).normal()));
    }

};

template <size_t DIMENSION>
const float Segmentator<DIMENSION>::sAngleThreshold = std::cos(AngleUtils::deg2rad(20.0f));

template class Segmentator<3>;

typedef Segmentator<3> Segmentator3d;

#endif // SEGMENTATOR_H
