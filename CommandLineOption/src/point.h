#ifndef POINT_H
#define POINT_H

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include <Eigen/Core>

template <size_t DIMENSION>
class Point
{
public:
    typedef Eigen::Matrix<float, DIMENSION, 1> Vector;

    Point(const Vector &position, const Eigen::Vector3f &color, float intensity, const Vector &normal, float normalConfidence, float curvature)
        : mPosition(position)
        , mColor(color)
        , mIntensity(intensity)
        , mNormal(normal)
        , mInvNormal(1 / mNormal.array())
        , mNormalConfidence(normalConfidence)
        , mCurvature(curvature)
        , mAngle((std::atan2(mNormal.y(), mNormal.x()) + M_PI) * 180.0f / M_PI)
    {
        static_assert(DIMENSION > 0, "Dimension must be greater than zero.");
    }

    Point(const Vector &position, const Eigen::Vector3f &color, float intensity, const Vector &normal, float normalConfidence)
        : Point(position, color, intensity, normal, normalConfidence, 0)
    {

    }

    Point(const Vector &position, const Eigen::Vector3f &color, float intensity, const Vector &normal)
        : Point(position, color, intensity, normal, 0, 0)
    {

    }

    Point(const Vector &position, const Eigen::Vector3f &color, float intensity)
        : Point(position, color, intensity, Vector::Zero(), 0, 0)
    {

    }

    Point(const Vector &position, const Eigen::Vector3f &color)
        : Point(position, color, 0, Vector::Zero(), 0, 0)
    {

    }

    Point(const Vector &position)
        : Point(position, Eigen::Vector3f::Zero(), 0, Vector::Zero(), 0, 0)
    {

    }

    Point()
        : Point(Vector::Zero(), Eigen::Vector3f::Zero(), 0, Vector::Zero(), 0, 0)
    {

    }

    const Vector& position() const
    {
        return mPosition;
    }

    void position(const Vector &position)
    {
        mPosition = position;
    }

    const Eigen::Vector3f& color() const
    {
        return mColor;
    }

    void color(const Eigen::Vector3f &color)
    {
        mColor = color;
    }

    float intensity() const
    {
        return mIntensity;
    }

    void intensity(float intensity)
    {
        mIntensity = intensity;
    }

    const Vector& normal() const
    {
        return mNormal;
    }

    void normal(const Vector& normal)
    {
        mNormal = normal;
        mInvNormal = 1 / mNormal.array();
        mAngle = (std::atan2(mNormal.y(), mNormal.x()) + M_PI) * 180.0f / M_PI;
    }

    const Vector& invNormal() const
    {
        return mInvNormal;
    }

    float angle() const
    {
        return mAngle;
    }

    float normalConfidence() const
    {
        return mNormalConfidence;
    }

    void normalConfidence(float normalConfidence)
    {
        mNormalConfidence = normalConfidence;
    }

    float curvature() const
    {
        return mCurvature;
    }

    void curvature(float curvature)
    {
        mCurvature = curvature;
    }

    Point<DIMENSION>& operator+=(const Point<DIMENSION> &other)
    {
        this->mPosition += other.mPosition;
        this->mColor += other.mColor;
        this->mIntensity += other.mIntensity;
        this->mNormal += other.mNormal;
        this->mNormalConfidence += other.mNormalConfidence;
        this->mCurvature += other.mCurvature;
        return *this;
    }

    Point<DIMENSION>& operator/=(float n)
    {
        this->mPosition /= n;
        this->mColor /= n;
        this->mIntensity /= n;
        this->mNormal /= n;
        this->mNormalConfidence /= n;
        this->mCurvature /= n;
        return *this;
    }

private:
    Vector mPosition;
    Eigen::Vector3f mColor;
    float mIntensity;
    Vector mNormal;
    Vector mInvNormal;
    float mNormalConfidence;
    float mCurvature;
    float mAngle;

};

template class Point<2>;
template class Point<3>;

typedef Point<2> Point2d;
typedef Point<3> Point3d;

#endif // POINT_H
