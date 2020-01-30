#ifndef PCACALCULATOR_H
#define PCACALCULATOR_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

template <size_t DIMENSION>
class PCACalculator
{
public:
    enum Method
    {
        SLOW = 0,
        FAST = 1
    };

    static void calculate(const Eigen::Matrix<float, DIMENSION, -1> &matrix,
                          Eigen::Matrix<float, DIMENSION, 1> &mean,
                          Eigen::Matrix<float, DIMENSION, DIMENSION> &eigenVectors,
                          Eigen::Matrix<float, DIMENSION, 1> &eigenValues,
                          Method method = SLOW)
    {
        mean = matrix.rowwise().mean();
        Eigen::Matrix<float, -1, DIMENSION> meanCentered = (matrix.colwise() - mean).transpose();
        if (method == SLOW)
        {
            Eigen::JacobiSVD<Eigen::Matrix<float, -1, DIMENSION> > svd(meanCentered, Eigen::ComputeFullV);
            Eigen::Matrix<float, DIMENSION, DIMENSION> v = svd.matrixV();
            Eigen::Matrix<float, DIMENSION, 1> s = svd.singularValues();
            eigenVectors = v;
            eigenValues = s;
        }
        else
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, -1, DIMENSION> > solver(meanCentered, Eigen::ComputeFullV);
            Eigen::Matrix<float, DIMENSION, DIMENSION> v = solver.eigenvectors();
            Eigen::Matrix<float, DIMENSION, 1> s = solver.eigenvalues();
            eigenVectors = v;//.rowwise().reverse();
            eigenValues = s;//.colwise().reverse();
        }
    }

    static void calculate(const Eigen::Matrix<float, DIMENSION, -1> &matrix,
                          Eigen::Matrix<float, DIMENSION, DIMENSION> &eigenVectors,
                          Eigen::Matrix<float, DIMENSION, 1> &eigenValues,
                          Method method = SLOW)
    {
        Eigen::Matrix<float, DIMENSION, 1> mean;
        calculate(matrix, mean, eigenVectors, eigenValues, method);
    }

    static void calculate(const Eigen::Matrix<float, DIMENSION, -1> &matrix,
                          Eigen::Matrix<float, DIMENSION, 1> &eigenValues,
                          Method method = SLOW)
    {
        Eigen::Matrix<float, DIMENSION, 1> mean;
        Eigen::Matrix<float, DIMENSION, DIMENSION> eigenVectors;
        calculate(matrix, mean, eigenVectors, eigenValues, method);
    }

    static void calculate(const Eigen::Matrix<float, DIMENSION, -1> &matrix,
                          Eigen::Matrix<float, DIMENSION, DIMENSION> &eigenVectors,
                          Method method = SLOW)
    {
        Eigen::Matrix<float, DIMENSION, 1> mean;
        Eigen::Matrix<float, DIMENSION, 1> eigenValues;
        calculate(matrix, mean, eigenVectors, eigenValues, method);
    }

};

template class PCACalculator<3>;

typedef PCACalculator<3> PCACalculator3d;

#endif // PCACALCULATOR_H
