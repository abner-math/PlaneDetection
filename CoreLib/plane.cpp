#include "plane.h"

#include <iostream>
#include "pcacalculator.h"

void Plane::leastSquares(const Eigen::Matrix3Xf &points)
{
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    PCACalculator3d::calculate(points, eigenVectors, eigenValues);
    mNormal = eigenVectors.col(2);
    mBasisU = eigenVectors.col(0) * eigenValues(0);
    mBasisV = eigenVectors.col(1) * eigenValues(1);
    this->center(points.rowwise().mean());
}
