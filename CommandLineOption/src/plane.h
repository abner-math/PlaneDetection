#ifndef PLANE_H
#define PLANE_H

#include "primitive.h"
#include "geometryutils.h"

class Plane : public Primitive3d
{
public:
    Plane()
        : Primitive3d(Eigen::Vector3f::Zero())
    {

    }

    Plane(const Eigen::Vector3f &center, const Eigen::Vector3f &normal, Eigen::Vector3f basisU = Eigen::Vector3f::Zero(), Eigen::Vector3f basisV = Eigen::Vector3f::Zero())
        : Primitive3d(center)
        , mNormal(normal)
        , mBasisU(basisU)
        , mBasisV(basisV)
    {
        mDistanceFromOrigin = -mNormal.dot(center);
    }

    const Eigen::Vector3f& normal() const
    {
        return mNormal;
    }

    void normal(const Eigen::Vector3f &normal)
    {
        mNormal = normal;
        mDistanceFromOrigin = -mNormal.dot(Primitive3d::center());
    }

    const Eigen::Vector3f& basisU() const
    {
        return mBasisU;
    }

    void basisU(const Eigen::Vector3f &basisU)
    {
        mBasisU = basisU;
    }

    const Eigen::Vector3f& basisV() const
    {
        return mBasisV;
    }

    void basisV(const Eigen::Vector3f &basisV)
    {
        mBasisV = basisV;
    }

    const Eigen::Vector3f& center() const override
    {
        return Primitive3d::center();
    }

    void center(const Eigen::Vector3f &center) override
    {
        Primitive3d::center(center);
        mDistanceFromOrigin = -mNormal.dot(Primitive3d::center());
    }

    float distanceFromOrigin() const
    {
        return mDistanceFromOrigin;
    }

    float getSignedDistanceFromSurface(const Eigen::Vector3f &point) const override
    {
        return mNormal.dot(point) + mDistanceFromOrigin;
    }

    Eigen::Vector3f normalAt(const Eigen::Vector3f &point) const override
    {
        return mNormal;
    }

    void leastSquares(const Eigen::Matrix3Xf &points) override;

private:
    Eigen::Vector3f mNormal;
    Eigen::Vector3f mBasisU;
    Eigen::Vector3f mBasisV;
    float mDistanceFromOrigin;

};

//template class Plane<float>;
//template class Plane<double>;

#endif // PLANE_H
