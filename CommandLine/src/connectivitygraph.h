#ifndef CONNECTIVITYGRAPH_H
#define CONNECTIVITYGRAPH_H

#include <string>
#include <map>
#include <set>
#include <queue>
#include <vector>

#include <Eigen/Core>

class ConnectivityGraph
{
public:
    ConnectivityGraph(size_t numNodes);

    void addNode(size_t node, const std::vector<size_t> &neighbors);

    std::vector<size_t> neighbors(size_t node) const
    {
        return std::vector<size_t>(mGraph.begin() + mGraphIndices[node].first, mGraph.begin() + mGraphIndices[node].first + mGraphIndices[node].second);
    }

    std::pair<std::vector<size_t>::const_iterator, std::vector<size_t>::const_iterator> neighborsIterator(size_t node) const {
        return std::make_pair(mGraph.begin() + mGraphIndices[node].first, mGraph.begin() + mGraphIndices[node].first + mGraphIndices[node].second);
    }

    void setGroupIndices(const std::vector<size_t> &indices);

    void addGroup(size_t group, const std::vector<size_t> &points);

    void removeGroup(size_t group);

    void mergeGroups(size_t sourceGroup, size_t destGroup);

    const std::vector<size_t>& groupIndices() const
    {
        return mGroupIndices;
    }

    const std::vector<size_t>& pointsInGroup(size_t group) const
    {
        return mGroups.at(group);
    }

    std::set<size_t> groups() const
    {
        std::set<size_t> groups;
        for (const std::pair<size_t, std::vector<size_t> > &group : mGroups)
        {
            groups.insert(group.first);
        }
        return groups;
    }

    size_t groupOf(size_t node) const
    {
        return mGroupIndices[node];
    }

    size_t numGroups() const
    {
        return mGroups.size();
    }

    size_t numPoints() const
    {
        return mGraphIndices.size();
    }

    size_t numPointsInGroup(size_t group) const
    {
        return mGroups.at(group).size();
    }

    bool hasGroups() const
    {
        return mGroupInitialized;
    }

private:
    std::vector<size_t> mGraph;
    std::vector<std::pair<size_t, size_t> > mGraphIndices;
    std::vector<size_t> mGroupIndices;
    std::map<size_t, std::vector<size_t> > mGroups;
    bool mGroupInitialized;

};

#endif // CONNECTIVITYGRAPH_H
