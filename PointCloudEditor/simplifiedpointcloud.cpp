#include "simplifiedpointcloud.h"

SimplifiedPointCloud::SimplifiedPointCloud(const PointCloud3d *pointCloud, size_t levels, size_t minNumPoints)
    : mPointCloud(pointCloud)
    , mLevels(levels)
    , mMinNumPoints(minNumPoints)
{
    update();
}

Point3d SimplifiedPointCloud::getAveragePoint(const Octree *node) const
{
    Point3d point;
    for (size_t &index : node->points())
    {
        point += mPointCloud->at(index);
    }
    if (!node->points().empty())
    {
        point /= node->points().size();
    }
    point.normal(point.normal().normalized());
    return point;
}

void SimplifiedPointCloud::addPoints(const Octree *node)
{
    if (node->isLeaf())
    {
        if (mPointCloud->hasConnectivity())
        {
            std::map<size_t, int> countIndices;
            for (const size_t &index : node->points())
            {
                countIndices[mPointCloud->connectivity()->groupOf(index)] += 1;
            }
            size_t maxGroup = 0;
            int maxGroupCount = 0;
            for (const std::pair<size_t, int> &p : countIndices)
            {
                if (p.second > maxGroupCount)
                {
                    maxGroupCount = p.second;
                    maxGroup = p.first;
                }
            }
            mGroupIndices.push_back(maxGroup);
        }
        else
        {
            mGroupIndices.push_back(0);
        }
        Point3d averagePoint = getAveragePoint(node);
        for (const size_t &point : node->points())
        {
            mReal2Virtual[point] = this->size();
        }
        mVirtual2Real.push_back(node->points());
        this->add(averagePoint);
    }
    else
    {
        for (const Partitioner3d *child : node->children())
        {
            addPoints((Octree*)child);
        }
    }
}

void SimplifiedPointCloud::update()
{
    mReal2Virtual.clear();
    mVirtual2Real.clear();
    mGroupIndices.clear();
    mPoints.clear();
    Octree octree(mPointCloud);
    octree.partition(mLevels, mMinNumPoints);
    addPoints(&octree);
    if (!mPointCloud->hasConnectivity())
    {
        PointCloud3d::connectivity(NULL);
    }
    else
    {
        ConnectivityGraph *graph = new ConnectivityGraph(mGroupIndices.size());
        graph->setGroupIndices(mGroupIndices);
        PointCloud3d::connectivity(graph);
    }
    PointCloud3d::update();
    this->mode(mPointCloud->mode());
}

