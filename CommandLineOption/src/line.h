#ifndef LINE_H
#define LINE_H

#include "point.h"

template <size_t DIMENSION>
class Line
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    Line()
    {

    }

    Line(const Vector &p1, const Vector &p2, bool segment = false)
        : mP1(p1)
        , mP2(p2)
        , mSegment(segment)
    {

    }

    Vector& p1()
    {
        return mP1;
    }

    Vector& p2()
    {
        return mP2;
    }

    bool& segment()
    {
        return mSegment;
    }

    float length()
    {
        return (mP1 - mP2).norm();
    }

    Vector direction()
    {
        return (mP2 - mP1).normalized();
    }

    /**
     * @brief Find the closest point in the line to another point
     * @param point
     *      Point to which the closest point will be found
     * @return
     *      The closest point in the line
     */
    Vector closestPointToPoint(const Vector &point) const
    {
        Vector direction = (mP2 - mP1).normalized();
        float projection = point.dot(direction);
        Vector center = mP1 - direction * mP1.dot(direction);
        if (!mSegment)
        {
            return center + projection * direction;
        }
        else
        {
            float p1Projection = mP1.dot(direction);
            float p2Projection = mP2.dot(direction);
            if (projection < std::min(p1Projection, p2Projection))
            {
                return center + std::min(p1Projection, p2Projection) * direction;
            }
            else if (projection > std::max(p1Projection, p2Projection))
            {
                return center + std::max(p1Projection, p2Projection) * direction;
            }
            else
            {
                return center + projection * direction;
            }
        }
    }

    float distanceToPoint(const Vector &point) const
    {
        return (point - closestPointToPoint(point)).norm();
    }

private:
    Vector mP1;
    Vector mP2;
    bool mSegment;

};

template class Line<2>;
template class Line<3>;

typedef Line<2> Line2d;
typedef Line<3> Line3d;

#endif // LINE_H
