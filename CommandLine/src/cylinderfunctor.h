#ifndef CYLINDERFUNCTOR_H
#define CYLINDERFUNCTOR_H

#include "functor.h"

template <typename TYPE>
class CylinderFunctor : public Functor<TYPE>
{
public:
    CylinderFunctor(const Eigen::Matrix<TYPE, 3, -1> &points)
        : Functor<TYPE>(3, points.cols())
        , mPoints(points)
    {

    }

    int operator()(const Eigen::Matrix<TYPE, -1, 1> &params, Eigen::Matrix<TYPE, -1, 1> &residuals)
    {
        Eigen::Matrix<TYPE, 3, 1> axis = Eigen::Matrix<TYPE, 3, 1>(params(0), params(1), params(2)).normalized();
        Eigen::Matrix<TYPE, 3, 1> center(params(3), params(4), params(5));
        TYPE radius = std::abs(params(6));
        residuals = ((mPoints.colwise() - center).colwise().cross(axis).colwise().norm().array() - radius).pow(2);
        return 0;
    }

    int df(const Eigen::Matrix<TYPE, -1, 1> &params, Eigen::Matrix<TYPE, -1, -1> &jacobian)
    {
        Eigen::Matrix<TYPE, 3, 1> a(params(0), params(1), params(2)); // axis
        Eigen::Matrix<TYPE, 3, 1> c(params(3), params(4), params(5)); // center
        TYPE r = params(6);  // radius

        TYPE var1 = a.squaredNorm();
        TYPE var6 = std::pow(var1, static_cast<TYPE>(1.5));

        Eigen::Array<TYPE, 1, -1> x = mPoints.row(0).array();
        Eigen::Array<TYPE, 1, -1> y = mPoints.row(1).array();
        Eigen::Array<TYPE, 1, -1> z = mPoints.row(2).array();
        Eigen::Array<TYPE, 1, -1> var2 = a.x()*c.y() - a.x()*y - a.y()*c.x() + a.y()*x;
        Eigen::Array<TYPE, 1, -1> var3 = -a.x()*c.z() + a.x()*z + a.z()*c.x() - a.z()*x;
        Eigen::Array<TYPE, 1, -1> var4 = a.y()*c.z() - a.y()*z - a.z()*c.y() + a.z()*y;
        Eigen::Array<TYPE, 1, -1> var5 = (var2.pow(2) + var3.pow(2) + var4.pow(2)).sqrt();
        Eigen::Array<TYPE, 1, -1> var7 = std::sqrt(var1)*var5;
        Eigen::Array<TYPE, 1, -1> var8 = 2*(var5/std::sqrt(var1) - r);

        jacobian.col(0) = (((c.y() - y)*var2 + (z - c.z())*var3)/var7 - (a.x()*var5)/var6)*var8;
        jacobian.col(1) = (((x - c.x())*var2 + (c.z() - z)*var4)/var7 - (a.y()*var5)/var6)*var8;
        jacobian.col(2) = (((c.x() - x)*var3 + (y - c.y())*var4)/var7 - (a.z()*var5)/var6)*var8;
        jacobian.col(3) = ((a.z()*var3 - a.y()*var2)*var8)/var7;
        jacobian.col(4) = ((a.x()*var2 - a.z()*var4)*var8)/var7;
        jacobian.col(5) = ((a.y()*var4 - a.x()*var3)*var8)/var7;
        jacobian.col(6) = -var8;

        return 0;
    }

private:
    Eigen::Matrix<TYPE, 3, -1> mPoints;

};

template class CylinderFunctor<float>;

#endif // CYLINDERFUNCTOR_H
