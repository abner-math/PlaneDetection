#ifndef BOUNDARYVOLUMEHIERARCHY_H
#define BOUNDARYVOLUMEHIERARCHY_H

#include <algorithm>
#include <set>
#include <iostream>

#include "partitioner.h"

template <size_t DIMENSION>
class BoundaryVolumeHierarchy : public Partitioner<DIMENSION>
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;
    static const size_t NUM_CHILDREN = 1 << DIMENSION;

    BoundaryVolumeHierarchy(const PointCloud<DIMENSION> *pointCloud)
        : Partitioner<DIMENSION>(pointCloud)
        , mRoot(this)
        , mParent(this)
        , mLeaf(true)
        , mLevel(0)
    {
        Rect<DIMENSION> extension = pointCloud->extension();
        mCenter = extension.center();
        mSize = extension.maxSize() / 2;
        mIndices = std::vector<size_t>(pointCloud->size());
        std::iota(mIndices.begin(), mIndices.end(), 0);
        mLeafTable = std::vector<BoundaryVolumeHierarchy<DIMENSION>*>(pointCloud->size(), this);
    }

    BoundaryVolumeHierarchy(const BoundaryVolumeHierarchy &bvh) = delete;

    ~BoundaryVolumeHierarchy()
    {
        if (!isLeaf())
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                {
                    delete mChildren[i];
                }
            }
        }
    }

    void partition(size_t levels = 1, size_t minNumPoints = 1, float minSize = 0.0f) override
    {
        if (!isLeaf())
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                    mChildren[i]->partition(levels - 1, minNumPoints, minSize);
            }
        }
        else
        {
            if (levels <= 0 || mIndices.size() <= minNumPoints || mIndices.size() <= 1 || mSize < minSize) return;
            // create new centers
            Vector newCenters[NUM_CHILDREN];
            calculateNewCenters(newCenters);

            mLeaf = false;

            // create children
            float newSize = mSize / 2;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                mChildren[i] = NULL;
            }

            // split points
            for (const size_t &index : mIndices)
            {
                // calculate child index comparing position to child center
                size_t childIndex = calculateChildIndex(this->pointCloud()->at(index).position());
                if (mChildren[childIndex] == NULL)
                {
                    mChildren[childIndex] = new BoundaryVolumeHierarchy<DIMENSION>(this, newCenters[childIndex], newSize);
                    mChildren[childIndex]->mIndices.reserve(mIndices.size());
                }
                mChildren[childIndex]->mIndices.push_back(index);
                // update current leaf where point is stored
                mRoot->mLeafTable[index] = mChildren[childIndex];
            }

            mIndices.clear();

            // partition recursively
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL) {
                    mChildren[i]->partition(levels - 1, minNumPoints, minSize);
                }
            }
        }
    }

    const Partitioner<DIMENSION>* getContainingLeaf(size_t index) const override
    {
        return mRoot->mLeafTable[index];
    }

    BoundaryVolumeHierarchy<DIMENSION>* child(size_t index)
    {
        return mChildren[index];
    }

    std::vector<const Partitioner<DIMENSION>*> children() const override
    {
        if (isLeaf()) return std::vector<const Partitioner<DIMENSION>*>();
        std::vector<const Partitioner<DIMENSION>*> children;
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            if (mChildren[i] != NULL)
                children.push_back(mChildren[i]);
        }
        return children;
    }

    const Partitioner<DIMENSION>* parent() const override
    {
        return mParent;
    }

    const Vector& center() const
    {
        return mCenter;
    }

    float cellSize() const
    {
        return mSize;
    }

    bool isRoot() const override
    {
        return this == mRoot;
    }

    bool isLeaf() const override
    {
        return mLeaf;
    }

    size_t octreeLevel() const
    {
        return mLevel;
    }

    size_t numPoints() const override
    {
        if (isLeaf())
        {
            return mIndices.size();
        }
        else
        {
            size_t numPoints = 0;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                    numPoints += mChildren[i]->numPoints();
            }
            return numPoints;
        }
    }

    Rect<DIMENSION> extension() const override
    {
        return Rect<DIMENSION>(mCenter - Vector::Constant(mSize),
                            mCenter + Vector::Constant(mSize));
    }

    std::vector<size_t> points() const override
    {
        if (isLeaf())
        {
            return mIndices;
        }
        else
        {
            std::vector<size_t> points;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                {
                    std::vector<size_t> childPoints = mChildren[i]->points();
                    points.insert(points.end(), childPoints.begin(), childPoints.end());
                }
            }
            return points;
        }
    }

    void getNeighborCells(std::vector<BoundaryVolumeHierarchy<DIMENSION>*> &neighbors)
    {
        BoundaryVolumeHierarchy<DIMENSION>* neighbor;
        for (size_t i = 0; i < DIMENSION; i++)
        {
            neighbor = getNeighborCellsGreaterOrEqual(i, false);
            getNeighborCellsSmaller(i, false, neighbor, neighbors);
            neighbor = getNeighborCellsGreaterOrEqual(i, true);
            getNeighborCellsSmaller(i, true, neighbor, neighbors);
        }
    }

private:
    BoundaryVolumeHierarchy *mRoot;
    BoundaryVolumeHierarchy *mParent;
    BoundaryVolumeHierarchy *mChildren[NUM_CHILDREN];
    Vector mCenter;
    float mSize;
    bool mLeaf;
    size_t mLevel;
    std::vector<size_t> mIndices;
    std::vector<BoundaryVolumeHierarchy<DIMENSION>*> mLeafTable;

    BoundaryVolumeHierarchy(BoundaryVolumeHierarchy *parent, const Vector &center, float size)
        : Partitioner<DIMENSION>(parent->pointCloud())
        , mRoot(parent->mRoot)
        , mParent(parent)
        , mCenter(center)
        , mSize(size)
        , mLeaf(true)
        , mLevel(parent->mLevel + 1)
    {
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            mChildren[i] = NULL;
        }
    }

    void calculateNewCenters(Vector centers[NUM_CHILDREN])
    {
        float newSize = mSize / 2;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                int signal = (((i & (1 << (DIMENSION - dim - 1))) >> (DIMENSION - dim - 1)) << 1) - 1;
                centers[i](dim) = mCenter(dim) + newSize * signal;
            }
        }
    }

    size_t calculateChildIndex(const Vector &position)
    {
        size_t childIndex = 0;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            childIndex |= (position(dim) > mCenter(dim)) << (DIMENSION - dim - 1);
        }
        return childIndex;
    }

    size_t getIndexOnParent()
    {
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            if (mParent->mChildren[i] == this)
            {
                return i;
            }
        }
        throw "Could not find node on parent";
    }

    size_t getIncrement(size_t direction, bool greaterThan)
    {
        int increment = 1 << (DIMENSION - direction - 1);
        if (!greaterThan)
        {
            increment = NUM_CHILDREN - increment;
        }
        return increment;
    }

    BoundaryVolumeHierarchy<DIMENSION>* getNeighborCellsGreaterOrEqual(size_t direction, bool greaterThan)
    {
        if (isRoot())
        {
            return NULL;
        }
        else if (greaterThan && mParent->mCenter(direction) > mCenter(direction) ||
            !greaterThan && mParent->mCenter(direction) < mCenter(direction))
        {
            size_t index = getIndexOnParent();
            size_t increment = getIncrement(direction, greaterThan);
            return mParent->mChildren[(index + increment) % NUM_CHILDREN];
        }
        BoundaryVolumeHierarchy<DIMENSION>* node = mParent->getNeighborCellsGreaterOrEqual(direction, greaterThan);
        if (node == NULL || node->isLeaf())
        {
            return node;
        }
        size_t index = getIndexOnParent();
        size_t increment = getIncrement(direction, !greaterThan);
        return node->mChildren[(index + increment) % NUM_CHILDREN];
    }

    void getNeighborCellsSmaller(size_t direction, bool greaterThan, BoundaryVolumeHierarchy<DIMENSION>* neighbor,
                                 std::vector<BoundaryVolumeHierarchy<DIMENSION>*> &neighbors)
    {
        std::queue<BoundaryVolumeHierarchy<DIMENSION>*> candidates;
        if (neighbor != NULL) candidates.push(neighbor);
        while (candidates.size() > 0)
        {
            BoundaryVolumeHierarchy<DIMENSION>* front = candidates.front();
            candidates.pop();
            if (front->isLeaf())
            {
                neighbors.push_back(front);
            }
            else
            {
                for (size_t i = 0; i < NUM_CHILDREN; i++)
                {
                    if (front->mChildren[i] == NULL) continue;
                    if ((greaterThan && front->mCenter(direction) > front->mChildren[i]->mCenter(direction)) ||
                        (!greaterThan && front->mCenter(direction) < front->mChildren[i]->mCenter(direction)))
                    {
                        candidates.push(front->mChildren[i]);
                    }
                }
            }
        }
    }

};

template class BoundaryVolumeHierarchy<2>;
template class BoundaryVolumeHierarchy<3>;

typedef BoundaryVolumeHierarchy<2> Quadtree;
typedef BoundaryVolumeHierarchy<3> Octree;

#endif // BOUNDARYVOLUMEHIERARCHY_H
