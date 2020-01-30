#include "connectivitygraph.h"

ConnectivityGraph::ConnectivityGraph(size_t numNodes)
    : mGroupInitialized(false)
{
    mGraphIndices = std::vector<std::pair<size_t, size_t> >(numNodes);
    mGroupIndices = std::vector<size_t>(numNodes, 0);
}

void ConnectivityGraph::addNode(size_t node, const std::vector<size_t> &neighbors)
{
    mGraphIndices[node].first = mGraph.size();
    mGraphIndices[node].second = neighbors.size();
    mGraph.insert(mGraph.end(), neighbors.begin(), neighbors.end());
}

void ConnectivityGraph::setGroupIndices(const std::vector<size_t> &indices)
{
    mGroupInitialized = true;
    mGroupIndices = indices;
    for (size_t i = 0; i < indices.size(); i++)
    {
        mGroups[indices[i]].push_back(i);
    }
}

void ConnectivityGraph::addGroup(size_t group, const std::vector<size_t> &points)
{
    mGroupInitialized = true;
    if (mGroups.find(group) == mGroups.end())
    {
        mGroups[group] = points;
    }
    else
    {
        for (const size_t & point : points)
        {
            mGroups[group].push_back(point);
        }
    }
    for (const size_t &point : points)
    {
        mGroupIndices[point] = group;
    }
}

void ConnectivityGraph::removeGroup(size_t group)
{
    auto it = mGroups.find(group);
    if (it != mGroups.end())
    {
        for (const size_t &point : mGroups[group])
        {
            mGroupIndices[point] = 0;
        }
        mGroups.erase(it);
    }
}

void ConnectivityGraph::mergeGroups(size_t sourceGroup, size_t destGroup)
{
    for (const size_t &point : mGroups[sourceGroup])
    {
        mGroupIndices[point] = destGroup;
    }
    std::vector<size_t> points = mGroups[sourceGroup];
    removeGroup(sourceGroup);
    addGroup(destGroup, points);
}
