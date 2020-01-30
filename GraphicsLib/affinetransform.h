#ifndef AFFINETRANSFORM_H
#define AFFINETRANSFORM_H

#include <Eigen/Dense>

#include "angleutils.h"

class AffineTransform
{
public:
    AffineTransform();

    void translate(const Eigen::Vector3f &translation);

    void place(const Eigen::Vector3f &position);

    void rotate(float degrees, const Eigen::Vector3f &axis);

    void scale(const Eigen::Vector3f &scale);

    void absScale(const Eigen::Vector3f &scale);

    void reset();

    const Eigen::Matrix4f& matrix() const
    {
        return mTransformationMatrix;
    }

    const Eigen::Vector3f& position() const
    {
        return mTranslation;
    }

    const Eigen::Vector3f& scale() const
    {
        return mScale;
    }

private:
    Eigen::Matrix4f mTransformationMatrix;
    Eigen::Matrix4f mRotationMatrix;
    Eigen::Vector3f mScale;
    Eigen::Vector3f mTranslation;

    void updateModelMatrix();

};

#endif // AFFINETRANSFORM_H
