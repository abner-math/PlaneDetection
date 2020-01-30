#include "lightsource.h"

#include "drawutils.h"
#include "angleutils.h"

LightSource::LightSource()
    : SceneObject(SceneObject::Priority::HIGH)
    , mTarget(Eigen::Vector3f::Zero())
    , mAzimuth(0.0f)
    , mElevation(90.0f)
    , mDistance(1.0f)
    , mSize(0.1f)
    , mColor(Eigen::Vector3f::Constant(1))
{

}

Eigen::Vector3f LightSource::position() const
{
    Eigen::Vector3f direction = AngleUtils::sphericalToCartesian(Eigen::Vector3f(mAzimuth, mElevation, mDistance));
    return mTarget + direction;
}

void LightSource::create()
{
    //buffer().color(mColor);
    //DrawUtils::outlineSphere(buffer(), position(), size());
}
