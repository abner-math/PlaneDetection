#include "circle.h"

#include <unsupported/Eigen/NonLinearOptimization>

void Circle::leastSquares(const Eigen::Matrix2Xf &points)
{
    /*Eigen::VectorXf params(3);
    params << this->center(), this->radius();
    CircleFunctor<float> functor(points);
    Eigen::LevenbergMarquardt<CircleFunctor<float>, float> lm(functor);
    lm.minimize(params);
    this->center(Eigen::Vector2f(params(0), params(1)));
    this->radius(std::abs(params(2)));*/
}
