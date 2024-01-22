#ifndef CYLINDERCONNECTOREXTREMITY_H
#define CYLINDERCONNECTOREXTREMITY_H

#define PARALLEL_ANGLE_THRESHOLD std::cos(AngleUtils::deg2rad(20.0f));
#define ORTHOGONAL_ANGLE_THRESHOLD std::cos(AngleUtils::deg2rad(70.0f));
#define MAXIUM_RADIUS_RATIO 5.0f;

#include "angleutils.h"
#include "geometryutils.h"
#include "cylinder.h"
#include "collisiondetector.h"

class Extremity
{
public:
    Extremity(const Cylinder *cylinder, bool left)
        : mCylinder(cylinder)
        , mLeft(left)
    {
        if (left)
        {
            mPosition = cylinder->extremities().first;
            mOtherPosition = cylinder->extremities().second;
        }
        else
        {
            mPosition = cylinder->extremities().second;
            mOtherPosition = cylinder->extremities().first;
        }
    }

    bool isParallelTo(const Extremity &other, float &dist) const
    {
        if (mCylinder == other.mCylinder) return false;
        dist = std::abs(mPosition.dot(mCylinder->axis()) - other.mPosition.dot(mCylinder->axis()));
        bool isAngleParallel = std::abs(mCylinder->axis().dot(other.mCylinder->axis())) > PARALLEL_ANGLE_THRESHOLD;
        if (!isAngleParallel) return false;
        bool doCylindersCross = mCylinder->getSignedDistanceFromSurface(other.mCylinder->center()) < 0 &&
                other.mCylinder->getSignedDistanceFromSurface(mCylinder->center()) < 0;
        if (!doCylindersCross) return false;
        bool isRadiusRatioTooBig = std::max(mCylinder->radius(), other.mCylinder->radius()) /
                                    std::min(mCylinder->radius(), other.mCylinder->radius()) > MAXIUM_RADIUS_RATIO;
        if (isRadiusRatioTooBig) return false;
        return true;
    }

    bool isOrthogonalTo(const Extremity &other, float &dist) const
    {
        if (mCylinder == other.mCylinder) return false;
        bool isAngleOrthogonal = std::abs(mCylinder->axis().dot(other.mCylinder->axis())) < ORTHOGONAL_ANGLE_THRESHOLD;
        if (!isAngleOrthogonal) return false;
        bool isRadiusRatioTooBig = std::max(mCylinder->radius(), other.mCylinder->radius()) /
                                    std::min(mCylinder->radius(), other.mCylinder->radius()) > MAXIUM_RADIUS_RATIO;
        if (isRadiusRatioTooBig) return false;
        Line3d line(mPosition, mPosition + mCylinder->axis());
        Line3d otherLine(other.mPosition, other.mPosition + other.mCylinder->axis());
        Eigen::Vector3f intersection;
        CollisionDetector::rayRayCollision3d(line, otherLine, intersection);
        if ((intersection - mPosition).norm() > (intersection - mOtherPosition).norm())
        {
            return false;
        }
        Eigen::Vector3f otherIntersection;
        CollisionDetector::rayRayCollision3d(otherLine, line, otherIntersection);
        if ((otherIntersection - other.mPosition).norm() > (otherIntersection - other.mOtherPosition).norm())
        {
            return false;
        }
        dist = (intersection - mPosition).norm() + (otherIntersection - other.mPosition).norm();
        bool doCylindersCross = mCylinder->getSignedDistanceFromSurface(otherIntersection) < 0 &&
                                other.mCylinder->getSignedDistanceFromSurface(intersection) < 0;
        return doCylindersCross;
    }

    const Cylinder* cylinder() const
    {
        return mCylinder;
    }

    const Eigen::Vector3f& position() const
    {
        return mPosition;
    }

    bool left() const
    {
        return mLeft;
    }

    bool operator==(const Extremity &other) const
    {
        return mCylinder == other.mCylinder && mLeft == other.mLeft;
    }

private:
    Eigen::Vector3f mPosition;
    Eigen::Vector3f mOtherPosition;
    const Cylinder *mCylinder;
    bool mLeft;

};

#endif // CYLINDERCONNECTOREXTREMITY_H
