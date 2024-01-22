#ifndef FUNCTOR_H
#define FUNCTOR_H

#include <Eigen/Core>

// Generic functor
template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    const int mInputs, mValues;

    Functor() : mInputs(InputsAtCompileTime), mValues(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : mInputs(inputs), mValues(values) {}

    int inputs() const { return mInputs; }
    int values() const { return mValues; }
};

#endif // FUNCTOR_H
