#include "cylinder.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include "cylinderfunctor.h"

void Cylinder::setHeight(const Eigen::Matrix<float, 3, -1> &points)
{
    float maxProjection = -std::numeric_limits<float>::max();
    float minProjection = std::numeric_limits<float>::max();
    for (int i = 0; i < points.cols(); i++)
    {
        Eigen::Vector3f position = points.col(i);
        float projection = position.dot(axis());
        maxProjection = std::max(maxProjection, projection);
        minProjection = std::min(minProjection, projection);
    }
    Eigen::Vector3f newCenter = center() - center().dot(axis()) * axis();
    newCenter = newCenter + (minProjection + (maxProjection - minProjection) / 2) * axis();
    float newHeight = maxProjection - minProjection;
    this->center(newCenter);
    this->height(newHeight);
}

void Cylinder::leastSquares(const Eigen::Matrix<float, 3, -1> &points)
{
    Eigen::VectorXf params(7);
    params << this->axis(), this->center(), this->radius();
    CylinderFunctor<float> functor(points);
    Eigen::LevenbergMarquardt<CylinderFunctor<float>, float> lm(functor);
    lm.minimize(params);
    this->axis(Eigen::Vector3f(params(0), params(1), params(2)).normalized());
    this->center(Eigen::Vector3f(params(3), params(4), params(5)));
    this->radius(std::abs(params(6)));
    setHeight(points);
}
