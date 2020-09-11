#ifndef NEARESTNEIGHBORCALCULATOR_H
#define NEARESTNEIGHBORCALCULATOR_H

#include <algorithm>
#include <set>

#include "partitioner.h"
#include "geometryutils.h"

// reference article: http://www.ri.cmu.edu/pub_files/pub1/moore_andrew_1991_1/moore_andrew_1991_1.pdf
template <size_t DIMENSION>
class NearestNeighborCalculator
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;
    typedef const Partitioner<DIMENSION> Node;

    static std::vector<std::pair<size_t, float> > kNN(Node *partitioner, size_t origin, size_t k)
    {
        std::vector<std::pair<size_t, float> > nearestNeighbors;
        std::set<size_t> setNearestNeighbors;

        setNearestNeighbors.insert(origin);

        for (size_t currentK = 0; currentK < k; currentK++)
        {
            float closestDist = std::numeric_limits<float>::max();
            int closestIndex = -1;
            Vector originPoint = partitioner->pointCloud()->at(origin).position();

            // find nearest point within leaf which contains the origin point (first approximation)
            Node *currentNode = partitioner->getContainingLeaf(origin);
            do
            {
                searchInLeafNode(currentNode, originPoint, closestIndex, closestDist, setNearestNeighbors);
                currentNode = currentNode->parent();
            } while (closestIndex == -1 && !currentNode->isRoot());

            // no more valid points can be found
            if (closestIndex == -1) return nearestNeighbors;

            // go to the highest possible level
            while (!currentNode->isRoot() && isNodeAPossibleCandidate(currentNode, originPoint, closestDist))
            {
                currentNode = currentNode->parent();
            }

            // now start to search from top-down
            searchInIntermediaryNode(currentNode, originPoint, closestIndex, closestDist, setNearestNeighbors);

            if (closestIndex == -1) return nearestNeighbors;

            nearestNeighbors.push_back(std::make_pair(closestIndex, closestDist));
            setNearestNeighbors.insert(closestIndex);
        }

        return nearestNeighbors;
    }

private:

    static void searchInLeafNode(Node *node, const Vector &queryPoint, int &currentClosestIndex,
                                 float &currentClosestDist, const std::set<size_t> &nearestNeighbors)
    {
        for (size_t &index : node->points())
        {
            // skip previous nearest points
            if (nearestNeighbors.find(index) != nearestNeighbors.end()) continue;
            Vector position = node->pointCloud()->at(index).position();
            float distance = (queryPoint - position).norm();
            if (distance < currentClosestDist)
            {
                currentClosestDist = distance;
                currentClosestIndex = index;
            }
        }
    }

    static void searchInIntermediaryNode(Node *node, const Vector &queryPoint, int &currentClosestIndex,
                                         float &currentClosestDist, std::set<size_t> &nearestNeighbors)
    {
        if (node->isLeaf())
        {
            if (isNodeAPossibleCandidate(node, queryPoint, currentClosestDist))
            {
                searchInLeafNode(node, queryPoint, currentClosestIndex, currentClosestDist, nearestNeighbors);
            }
        }
        else
        {
            for (Node *child : node->children())
            {
                if (child != NULL && (child->isLeaf() || isNodeAPossibleCandidate(child, queryPoint, currentClosestDist)))
                {
                    searchInIntermediaryNode(child, queryPoint, currentClosestIndex, currentClosestDist, nearestNeighbors);
                }
            }
        }
    }

    static inline bool isNodeAPossibleCandidate(Node *node, const Vector &point, float currentClosestDist)
    {
        return node->extension().distanceToPoint(point) <= currentClosestDist;
    }

};

template class NearestNeighborCalculator<3>;

typedef NearestNeighborCalculator<3> NearestNeighborCalculator3d;

#endif // NEARESTNEIGHBORCALCULATOR_H
