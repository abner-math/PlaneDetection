#ifndef PARTITIONER_H
#define PARTITIONER_H

#include "pointcloud.h"

template <size_t DIMENSION>
class Partitioner
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    Partitioner(const PointCloud<DIMENSION> *pointCloud)
        : mPointCloud(pointCloud)
    {

    }

    virtual ~Partitioner()
    {

    }

    const PointCloud<DIMENSION>* pointCloud() const
    {
        return mPointCloud;
    }

    size_t level() const
    {
        if (isRoot())
        {
            return 0;
        }
        else
        {
            return 1 + parent()->level();
        }
    }

    size_t height() const
    {
        if (isLeaf())
        {
            return 0;
        }
        else
        {
            size_t maxHeight = 0;
            for (const Partitioner<DIMENSION> *child : children())
            {
                maxHeight = std::max(maxHeight, child->height());
            }
            return 1 + maxHeight;
        }
    }

    /**
     * @brief Find the leaf which contains the point
     * @param point
     *      Point to which the leaf will be searched upon
     * @return
     *      The leaf which contains the point
     */
    const Partitioner<DIMENSION>* findNearestLeaf(const Vector &point) const
    {
        if (isLeaf())
        {
            return this;
        }
        else
        {
            const Partitioner<DIMENSION>* nearest = NULL;
            float nearestDist = std::numeric_limits<float>::max();
            for (const Partitioner<DIMENSION> *child : children())
            {
                if (child != NULL)
                {
                    float dist = (point - child->extension().center()).squaredNorm();
                    if (dist < nearestDist)
                    {
                        nearestDist = dist;
                        nearest = child;
                    }
                }
            }
            return nearest->findNearestLeaf(point);
        }
    }

    virtual const Partitioner<DIMENSION>* getContainingLeaf(size_t index) const = 0;

    virtual void partition(size_t levels, size_t minNumPoints, float minSize) = 0;

    virtual std::vector<const Partitioner<DIMENSION>*> children() const = 0;

    virtual const Partitioner<DIMENSION>* parent() const = 0;

    virtual bool isRoot() const = 0;

    virtual bool isLeaf() const = 0;

    virtual Rect<DIMENSION> extension() const = 0;

    virtual std::vector<size_t> points() const = 0;

    virtual size_t numPoints() const = 0;

private:
    const PointCloud<DIMENSION>* mPointCloud;

};

template class Partitioner<2>;
template class Partitioner<3>;

typedef Partitioner<2> Partitioner2d;
typedef Partitioner<3> Partitioner3d;

#endif // PARTITIONER_H
