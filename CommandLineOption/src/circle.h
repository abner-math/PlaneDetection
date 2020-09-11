#ifndef CIRCLE_H
#define CIRCLE_H

#include "primitive.h"
#include "geometryutils.h"

class Circle : public Primitive2d
{
public:
    Circle(const Eigen::Vector2f &center, float radius)
        : Primitive2d(center)
        , mRadius(radius)
    {

    }

    float radius() const
    {
        return mRadius;
    }

    void radius(float radius)
    {
        mRadius = radius;
    }

    float getSignedDistanceFromSurface(const Eigen::Vector2f &point) const override
    {
        return (point - this->center()).norm() - mRadius;
    }

    Eigen::Vector2f normalAt(const Eigen::Vector2f &point) const override
    {
        return (point - this->center()).normalized();
    }

    bool containsRect(Rect2d rect) const
    {
        std::vector<Eigen::Vector2f> vertices;
        rect.getVertices(vertices);
        for (size_t i = 0; i < vertices.size(); i++)
        {
            if ((vertices[i] - Primitive2d::center()).norm() > mRadius) return false;
        }
        return true;
    }

    void leastSquares(const Eigen::Matrix2Xf &points) override;

private:
    float mRadius;

};

#endif // CIRCLE_H
