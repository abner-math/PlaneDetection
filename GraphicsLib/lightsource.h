#ifndef LIGHTSOURCE_H
#define LIGHTSOURCE_H

#include "sceneobject.h"

class LightSource : public SceneObject
{
public:
    LightSource();

    Eigen::Vector3f& target()
    {
        return mTarget;
    }

    float& azimuth()
    {
        return mAzimuth;
    }

    float& elevation()
    {
        return mElevation;
    }

    float& distance()
    {
        return mDistance;
    }

    float& size()
    {
        return mSize;
    }

    Eigen::Vector3f& color()
    {
        return mColor;
    }

    Eigen::Vector3f position() const;

private:
    Eigen::Vector3f mTarget;
    float mAzimuth;
    float mElevation;
    float mDistance;
    float mSize;
    Eigen::Vector3f mColor;

    void create() override;

};

#endif // LIGHTSOURCE_H
