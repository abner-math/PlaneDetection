#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <set>
#include <vector>

#include <Eigen/Core>

template <size_t DIMENSION>
class Primitive
{
public:
    Primitive(const Eigen::Matrix<float, DIMENSION, 1> &center)
        : mCenter(center)
    {

    }

    virtual ~Primitive()
    {

    }

    virtual const Eigen::Matrix<float, DIMENSION, 1>& center() const
    {
        return mCenter;
    }

    virtual void center(const Eigen::Matrix<float, DIMENSION, 1> &center)
    {
        mCenter = center;
    }

    const Eigen::Vector3f& color() const
    {
        return mColor;
    }

    void color(const Eigen::Vector3f &color)
    {
        mColor = color;
    }

    int label() const
    {
        return mLabel;
    }

    void label(int label)
    {
        mLabel = label;
    }

    const std::vector<size_t>& inliers() const
    {
        return mInliers;
    }

    void inliers(const std::vector<size_t> &inliers)
    {
        mInliers = inliers;
    }

    void addInlier(size_t point)
    {
        mInliers.push_back(point);
    }

    void addInliers(const std::vector<size_t> &points)
    {
        mInliers.insert(mInliers.end(), points.begin(), points.end());
    }

    virtual void leastSquares(const Eigen::Matrix<float, DIMENSION, -1> &points)
    {
        
    }

    virtual float getSignedDistanceFromSurface(const Eigen::Matrix<float, DIMENSION, 1> &point) const = 0;

    virtual Eigen::Matrix<float, DIMENSION, 1> normalAt(const Eigen::Matrix<float, DIMENSION, 1> &point) const = 0;

protected:
    Eigen::Matrix<float, DIMENSION, 1> mCenter;
    Eigen::Vector3f mColor;
    std::vector<size_t> mInliers;
    int mLabel;

};

template class Primitive<2>;
template class Primitive<3>;

typedef Primitive<2> Primitive2d;
typedef Primitive<3> Primitive3d;

#endif // PRIMITIVE_H
