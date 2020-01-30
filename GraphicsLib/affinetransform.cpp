#include "affinetransform.h"

AffineTransform::AffineTransform()
    : mTransformationMatrix(Eigen::Matrix4f::Identity())
    , mRotationMatrix(Eigen::Matrix4f::Identity())
    , mScale(Eigen::Vector3f::Constant(1.0f))
    , mTranslation(Eigen::Vector3f::Zero())
{

}

void AffineTransform::translate(const Eigen::Vector3f &translation)
{
    mTranslation += translation;
    updateModelMatrix();
}

void AffineTransform::place(const Eigen::Vector3f &position)
{
    mTranslation = position;
    updateModelMatrix();
}

void AffineTransform::rotate(float degrees, const Eigen::Vector3f &axis)
{
    mRotationMatrix *= AngleUtils::quaternionMatrix(degrees, axis);
    updateModelMatrix();
}

void AffineTransform::scale(const Eigen::Vector3f &scale)
{
    mScale.x() *= scale.x();
    mScale.y() *= scale.y();
    mScale.z() *= scale.z();
    updateModelMatrix();
}

void AffineTransform::absScale(const Eigen::Vector3f &scale)
{
    mScale = scale;
    updateModelMatrix();
}


void AffineTransform::reset()
{
    mRotationMatrix = Eigen::Matrix4f::Identity();
    mScale = Eigen::Vector3f::Constant(1.0f);
    mTranslation = Eigen::Vector3f::Zero();
    mTransformationMatrix = Eigen::Matrix4f::Identity();
}

void AffineTransform::updateModelMatrix()
{
    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
    scaleMatrix(0, 0) = mScale.x();
    scaleMatrix(1, 1) = mScale.y();
    scaleMatrix(2, 2) = mScale.z();

    Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
    translationMatrix(0, 3) = mTranslation.x();
    translationMatrix(1, 3) = mTranslation.y();
    translationMatrix(2, 3) = mTranslation.z();

    mTransformationMatrix = mRotationMatrix * translationMatrix * scaleMatrix;
}
