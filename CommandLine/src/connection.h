#ifndef CYLINDERCONNECTORCONNECTION_H
#define CYLINDERCONNECTORCONNECTION_H

#include "extremity.h"

class Connection
{
public:
    enum Type
    {
        UNION = 0,
        ELBOW = 1
    };

    Connection(Rect3d volume)
        : mVolume(volume)
    {

    }

    void addExtremity(Extremity extremity)
    {
        mExtremities.push_back(extremity);
    }

    Rect3d volume() const
    {
        return mVolume;
    }

    const std::vector<Extremity>& extremities() const
    {
        return mExtremities;
    }

    Type type() const
    {
        return mType;
    }

    void type(Type type)
    {
        mType = type;
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

private:
    Rect3d mVolume;
    std::vector<Extremity> mExtremities;
    Type mType;
    std::vector<size_t> mInliers;

};

#endif // CYLINDERCONNECTORCONNECTION_H
