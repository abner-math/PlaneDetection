#ifndef CYLINDER_H
#define CYLINDER_H

#include "primitive.h"
#include "geometryutils.h"

class Cylinder : public Primitive3d
{
public:
    Cylinder(const Eigen::Vector3f &center, const Eigen::Vector3f &axis, float radius, float height = 0)
        : Primitive3d(center)
        , mAxis(axis)
        , mRadius(radius)
        , mHeight(height)
    {

    }

    const Eigen::Vector3f& axis() const
    {
        return mAxis;
    }

    void axis(const Eigen::Vector3f &axis)
    {
        mAxis = axis;
    }

    float radius() const
    {
        return mRadius;
    }

    void radius(float radius)
    {
        mRadius = radius;
    }

    float height() const
    {
        return mHeight;
    }

    void height(float height)
    {
        mHeight = height;
    }

    float getSignedDistanceFromSurface(const Eigen::Vector3f &point) const override
    {
        return Line3d(this->center(), this->center() + mAxis).distanceToPoint(point) - mRadius;
    }

    Eigen::Vector3f normalAt(const Eigen::Vector3f &point) const override
    {
        return (point - Line3d(this->center(), this->center() + mAxis).closestPointToPoint(point)).normalized();
    }

    std::pair<Eigen::Vector3f, Eigen::Vector3f> extremities() const
    {
        return std::make_pair(this->center() + mAxis * mHeight / 2, this->center() - mAxis * mHeight / 2);
    }

    bool containsRect(Rect3d rect) const
    {
        std::vector<Eigen::Vector3f> vertices;
        rect.getVertices(vertices);
        Line3d axis(Primitive3d::center(), Primitive3d::center() + mAxis);
        for (size_t i = 0; i < vertices.size(); i++)
        {
            Eigen::Vector3f closestPoint = axis.closestPointToPoint(vertices[i]);
            if ((vertices[i] - closestPoint).norm() > mRadius) return false;
        }
        return true;
    }

    void setHeight(const Eigen::Matrix<float, 3, -1> &points);

    void leastSquares(const Eigen::Matrix<float, 3, -1> &points) override;

private:
    Eigen::Vector3f mAxis;
    float mRadius;
    float mHeight;

};

#endif // CYLINDER_H
