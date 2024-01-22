#ifndef CUBE_H
#define CUBE_H

#include <vector>

#include "point.h"

template <size_t DIMENSION>
class Rect
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    Rect()
    {

    }

    Rect(const Vector &bottomLeft, const Vector &topRight)
        : mBottomLeft(bottomLeft)
        , mTopRight(topRight)
    {

    }

    Vector& bottomLeft()
    {
        return mBottomLeft;
    }

    Vector& topRight()
    {
        return mTopRight;
    }

    Vector center() const
    {
        return (mBottomLeft + mTopRight) / 2;
    }

    float maxSize() const
    {
        float maxSize = 0;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            maxSize = std::max(maxSize, mTopRight(dim) - mBottomLeft(dim));
        }
        return maxSize;
    }

    float minSize() const
    {
        float minSize = std::numeric_limits<float>::max();
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            minSize = std::min(minSize, mTopRight(dim) - mBottomLeft(dim));
        }
        return minSize;
    }

    float width() const
    {
        return mTopRight(0) - mBottomLeft(0);
    }

    float height() const
    {
        return mTopRight(1) - mBottomLeft(1);
    }

    float depth() const
    {
        return mTopRight(2) - mBottomLeft(2);
    }

    void getVertices(std::vector<Vector> &vertices) const
    {
        size_t numVertices = 1 << DIMENSION;
        vertices.resize(numVertices);
        for (size_t i = 0; i < numVertices; i++)
        {
            for (size_t dim = 0; dim < DIMENSION; dim++)
            {
                int signal = (((i & (1 << dim)) >> dim) << 1) - 1;
                if (signal > 0)
                {
                    vertices[i](dim) = mTopRight(dim);
                }
                else
                {
                    vertices[i](dim) = mBottomLeft(dim);
                }
            }
        }
    }

    /**
     * @brief Find the closest point in the rect edges to point
     * @param point
     *      Point to which the closest point will be found
     * @return
     *      The closest point in the rect edges
     */
    Vector closestPointToPoint(const Vector &point) const
    {
        Vector closestPoint;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            if (point(dim) <= mBottomLeft(dim))
            {
                closestPoint(dim) = mBottomLeft(dim);
            }
            else if (point(dim) >= mTopRight(dim))
            {
                closestPoint(dim) = mTopRight(dim);
            }
            else
            {
                closestPoint(dim) = point(dim);
            }
        }
        return closestPoint;
    }

    float distanceToPoint(const Vector &point) const
    {
        return (point - closestPointToPoint(point)).norm();
    }

    bool containsPoint(const Vector &point) const
    {
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            float min = std::min(mBottomLeft(dim), mTopRight(dim));
            float max = std::max(mBottomLeft(dim), mTopRight(dim));
            if (!(point(dim) >= min && point(dim) <= max))
            {
                return false;
            }
        }
        return true;
    }

    bool operator==(const Rect &other) const
    {
        return mBottomLeft == other.mBottomLeft &&
                mTopRight == other.mTopRight;
    }

private:
    Vector mBottomLeft;
    Vector mTopRight;

};

template class Rect<2>;
template class Rect<3>;

typedef Rect<2> Rect2d;
typedef Rect<3> Rect3d;

#endif // CUBE_H
